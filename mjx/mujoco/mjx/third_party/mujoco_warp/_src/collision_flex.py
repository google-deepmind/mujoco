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

from typing import Tuple

import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src import collision_primitive_core
from mujoco.mjx.third_party.mujoco_warp._src.collision_core import Geom
from mujoco.mjx.third_party.mujoco_warp._src.collision_core import sap_range
from mujoco.mjx.third_party.mujoco_warp._src.collision_core import sap_sweep  # TODO(team): consolidate _flex_sap_project with geom _sap_project
from mujoco.mjx.third_party.mujoco_warp._src.collision_gjk import ccd
from mujoco.mjx.third_party.mujoco_warp._src.math import make_frame
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MAX_EPAFACES
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MAX_EPAHORIZON
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MAXVAL
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MINMU
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MINVAL
from mujoco.mjx.third_party.mujoco_warp._src.types import ContactType
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import GeomType
from mujoco.mjx.third_party.mujoco_warp._src.types import Model
from mujoco.mjx.third_party.mujoco_warp._src.types import OverflowType
from mujoco.mjx.third_party.mujoco_warp._src.types import vec5
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope

wp.set_module_options({"enable_backward": False})


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


@wp.kernel
def _flex_broadphase_bounds(
  # Model:
  flex_margin: wp.array[float],
  flex_gap: wp.array[float],
  flex_vertadr: wp.array[int],
  flex_vertnum: wp.array[int],
  flex_radius: wp.array[float],
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
  bound = flex_radius[flexid] + margin
  inflate = wp.vec3(bound, bound, bound)

  flex_aabb_min_out[worldid, flexid] = min_bound - inflate
  flex_aabb_max_out[worldid, flexid] = max_bound + inflate


@wp.func
def _flex_triangle_geom_broadphase(
  # Model:
  ngeom: int,
  opt_warn_overflow: bool,
  geom_type: wp.array[int],
  geom_aabb: wp.array3d[wp.vec3],
  geom_margin: wp.array2d[float],
  # Data in:
  geom_xpos_in: wp.array2d[wp.vec3],
  geom_xmat_in: wp.array2d[wp.mat33],
  naconmax_in: int,
  # In:
  worldid: int,
  element_or_shell_id: int,
  flexid: int,
  geomid: int,
  t1: wp.vec3,
  t2: wp.vec3,
  t3: wp.vec3,
  tri_radius: float,
  tri_margin: float,
  flex_aabb_min_val: wp.vec3,
  flex_aabb_max_val: wp.vec3,
  # Data out:
  ncollision_out: wp.array[int],
  # Data out:
  overflow_out: wp.array[int],
  # Out:
  collision_pair_out: wp.array[wp.vec2i],
  collision_worldid_out: wp.array[int],
):
  gtype = geom_type[geomid]
  if (
    gtype != int(GeomType.SPHERE)
    and gtype != int(GeomType.CAPSULE)
    and gtype != int(GeomType.BOX)
    and gtype != int(GeomType.CYLINDER)
    and gtype != int(GeomType.MESH)
    and gtype != int(GeomType.ELLIPSOID)
  ):
    return

  geom_margin_val = geom_margin[worldid % geom_margin.shape[0], geomid]
  margin = geom_margin_val + tri_margin

  aabb_id = worldid % geom_aabb.shape[0]
  geom_center_local = geom_aabb[aabb_id, geomid, 0]
  geom_half_size_local = geom_aabb[aabb_id, geomid, 1]

  geom_pos = geom_xpos_in[worldid, geomid]
  geom_rot = geom_xmat_in[worldid, geomid]

  # Stage 1 Filter: Coarse flex AABB vs Geom world AABB check
  # Transform center to global frame
  geom_center_global = geom_rot @ geom_center_local + geom_pos

  # Project local half-size onto world axes using absolute rotation matrix entries
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
    return

  # Stage 2 Filter: Element AABB vs Geom world AABB check
  tri_min = wp.min(t1, wp.min(t2, t3)) - wp.vec3(tri_radius, tri_radius, tri_radius)
  tri_max = wp.max(t1, wp.max(t2, t3)) + wp.vec3(tri_radius, tri_radius, tri_radius)

  if _flex_element_aabb_filter(geom_box_min, geom_box_max, tri_min, tri_max):
    return

  # Stage 3 Filter: Project Geom onto triangle plane normal
  normal = wp.normalize(wp.cross(t2 - t1, t3 - t1))
  signed_dist = wp.dot(geom_pos - t1, normal)

  r_extent = float(0.0)
  if gtype == int(GeomType.SPHERE):
    r_extent = geom_half_size_local[0]
  elif gtype == int(GeomType.CAPSULE):
    r_extent = geom_half_size_local[0] + geom_half_size_local[1]
  elif gtype == int(GeomType.CYLINDER):
    r_extent = wp.sqrt(geom_half_size_local[0] * geom_half_size_local[0] + geom_half_size_local[1] * geom_half_size_local[1])
  elif gtype == int(GeomType.BOX):
    r_extent = wp.length(geom_half_size_local)
  elif gtype == int(GeomType.MESH):
    r_extent = wp.length(geom_half_size_local)
  elif gtype == int(GeomType.ELLIPSOID):
    r_extent = wp.length(geom_half_size_local)

  if wp.abs(signed_dist) > r_extent + margin + tri_radius:
    return

  # Overlap found! Save candidate.
  idx = wp.atomic_add(ncollision_out, 0, 1)
  if idx >= naconmax_in:
    if opt_warn_overflow:
      wp.printf("Collision buffer overflow in flex broadphase - please increase naconmax to %u\n", idx + 1)
    wp.atomic_or(overflow_out, worldid, wp.static(OverflowType.BROADPHASE))
    return
  collision_pair_out[idx] = wp.vec2i(element_or_shell_id, geomid)
  collision_worldid_out[idx] = worldid


@wp.kernel
def _flex_broadphase_unified(
  # Model:
  ngeom: int,
  nflex: int,
  opt_warn_overflow: bool,
  geom_type: wp.array[int],
  geom_size: wp.array2d[wp.vec3],
  geom_aabb: wp.array3d[wp.vec3],
  geom_rbound: wp.array2d[float],
  geom_margin: wp.array2d[float],
  flex_margin: wp.array[float],
  flex_dim: wp.array[int],
  flex_vertadr: wp.array[int],
  flex_radius: wp.array[float],
  # Data in:
  geom_xpos_in: wp.array2d[wp.vec3],
  geom_xmat_in: wp.array2d[wp.mat33],
  flexvert_xpos_in: wp.array2d[wp.vec3],
  naconmax_in: int,
  flex_aabb_min_in: wp.array2d[wp.vec3],
  flex_aabb_max_in: wp.array2d[wp.vec3],
  # In:
  triadr: wp.array[int],
  tridataadr: wp.array[int],
  tri: wp.array[int],
  pairs_filtered: wp.array[wp.vec2i],
  triflexid: wp.array[int],
  # Data out:
  ncollision_out: wp.array[int],
  # Data out:
  overflow_out: wp.array[int],
  # Out:
  collision_pair_out: wp.array[wp.vec2i],
  collision_worldid_out: wp.array[int],
):
  worldid, pairid = wp.tid()

  pair = pairs_filtered[pairid]
  tri_id = pair[0]
  geomid = pair[1]

  flexid = triflexid[tri_id]

  vert_adr = flex_vertadr[flexid]
  tri_radius = flex_radius[flexid]
  tri_margin = flex_margin[flexid]

  tri_data_idx = tridataadr[flexid] + (tri_id - triadr[flexid]) * 3
  v0_local = tri[tri_data_idx]
  v1_local = tri[tri_data_idx + 1]
  v2_local = tri[tri_data_idx + 2]

  t1 = flexvert_xpos_in[worldid, vert_adr + v0_local]
  t2 = flexvert_xpos_in[worldid, vert_adr + v1_local]
  t3 = flexvert_xpos_in[worldid, vert_adr + v2_local]

  _flex_triangle_geom_broadphase(
    ngeom,
    opt_warn_overflow,
    geom_type,
    geom_aabb,
    geom_margin,
    geom_xpos_in,
    geom_xmat_in,
    naconmax_in,
    worldid,
    tri_id,
    flexid,
    geomid,
    t1,
    t2,
    t3,
    tri_radius,
    tri_margin,
    flex_aabb_min_in[worldid, flexid],
    flex_aabb_max_in[worldid, flexid],
    # Data out:
    ncollision_out,
    overflow_out,
    collision_pair_out,
    collision_worldid_out,
  )


@wp.kernel
def _flex_broadphase_plane(
  # Model:
  ngeom: int,
  opt_warn_overflow: bool,
  geom_type: wp.array[int],
  geom_margin: wp.array2d[float],
  flex_margin: wp.array[float],
  flex_vertadr: wp.array[int],
  flex_radius: wp.array[float],
  flexvert_geom_pair_filtered: wp.array[wp.vec2i],
  flex_vertflexid: wp.array[int],
  # Data in:
  geom_xpos_in: wp.array2d[wp.vec3],
  geom_xmat_in: wp.array2d[wp.mat33],
  flexvert_xpos_in: wp.array2d[wp.vec3],
  naconmax_in: int,
  flex_aabb_min_in: wp.array2d[wp.vec3],
  flex_aabb_max_in: wp.array2d[wp.vec3],
  # Data out:
  ncollision_out: wp.array[int],
  # Data out:
  overflow_out: wp.array[int],
  # Out:
  collision_pair_out: wp.array[wp.vec2i],
  collision_worldid_out: wp.array[int],
):
  worldid, pairid = wp.tid()

  pair = flexvert_geom_pair_filtered[pairid]
  vertid = pair[0]
  geomid = pair[1]

  flexid = flex_vertflexid[vertid]
  radius = flex_radius[flexid]
  flex_margin_val = flex_margin[flexid]

  vert = flexvert_xpos_in[worldid, vertid]

  flex_aabb_min = flex_aabb_min_in[worldid, flexid]
  flex_aabb_max = flex_aabb_max_in[worldid, flexid]

  gtype = geom_type[geomid]
  if gtype != int(GeomType.PLANE):
    return

  margin = geom_margin[worldid % geom_margin.shape[0], geomid] + flex_margin_val
  geom_pos = geom_xpos_in[worldid, geomid]
  geom_rot = geom_xmat_in[worldid, geomid]
  plane_normal = wp.vec3(geom_rot[0, 2], geom_rot[1, 2], geom_rot[2, 2])

  # Stage 1 filter: Bounding box of flex vs plane
  flex_center = 0.5 * (flex_aabb_min + flex_aabb_max)
  flex_half_size = 0.5 * (flex_aabb_max - flex_aabb_min)

  proj_half = (
    wp.abs(flex_half_size[0] * plane_normal[0])
    + wp.abs(flex_half_size[1] * plane_normal[1])
    + wp.abs(flex_half_size[2] * plane_normal[2])
  )

  diff_center = flex_center - geom_pos
  dist_center = wp.dot(diff_center, plane_normal)

  if dist_center - proj_half > margin:
    return

  diff = vert - geom_pos
  signed_dist = wp.dot(diff, plane_normal)
  dist = signed_dist - radius

  if dist < margin:
    # Append Candidate to Context
    idx = wp.atomic_add(ncollision_out, 0, 1)
    if idx >= naconmax_in:
      if opt_warn_overflow:
        wp.printf("Collision buffer overflow in flex plane broadphase - please increase naconmax to %u\n", idx + 1)
      wp.atomic_or(overflow_out, worldid, wp.static(OverflowType.BROADPHASE))
      return
    collision_pair_out[idx] = wp.vec2i(vertid, geomid)
    collision_worldid_out[idx] = worldid


@wp.func
def _write_flex_contact(
  # Model:
  geom_condim: wp.array[int],
  geom_priority: wp.array[int],
  geom_solmix: wp.array2d[float],
  geom_solref: wp.array2d[wp.vec2],
  geom_solimp: wp.array2d[vec5],
  geom_friction: wp.array2d[wp.vec3],
  geom_gap: wp.array2d[float],
  flex_condim: wp.array[int],
  flex_priority: wp.array[int],
  flex_solmix: wp.array[float],
  flex_solref: wp.array[wp.vec2],
  flex_solimp: wp.array[vec5],
  flex_friction: wp.array[wp.vec3],
  flex_margin: wp.array[float],
  flex_gap: wp.array[float],
  # Data in:
  naconmax_in: int,
  # In:
  collisionid: int,
  dist: float,
  pos: wp.vec3,
  normal: wp.vec3,
  margin: float,
  geomid: int,
  flexid: int,
  elemid: int,
  vertid: int,
  worldid: int,
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
  contact_worldid_out: wp.array[int],
  contact_type_out: wp.array[int],
  contact_geomcollisionid_out: wp.array[int],
  nacon_out: wp.array[int],
):
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

  c_idx = wp.atomic_add(nacon_out, 0, 1)
  if c_idx < naconmax_in:
    frame = make_frame(normal)

    contact_dist_out[c_idx] = dist
    contact_pos_out[c_idx] = pos
    contact_frame_out[c_idx] = frame
    contact_includemargin_out[c_idx] = margin
    contact_friction_out[c_idx] = friction
    contact_solref_out[c_idx] = solref
    contact_solreffriction_out[c_idx] = solref  # Same for flex contacts
    contact_solimp_out[c_idx] = solimp
    contact_dim_out[c_idx] = condim
    contact_geom_out[c_idx] = wp.vec2i(geomid, -1)
    contact_flex_out[c_idx] = wp.vec2i(-1, flexid)
    contact_elem_out[c_idx] = wp.vec2i(-1, elemid)
    contact_vert_out[c_idx] = wp.vec2i(-1, vertid)
    contact_worldid_out[c_idx] = worldid
    contact_type_out[c_idx] = ContactType.CONSTRAINT
    contact_geomcollisionid_out[c_idx] = collisionid


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
):
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
def _write_candidate_contact(
  # In:
  max_candidates: int,
  dist: float,
  pos: wp.vec3,
  nrm: wp.vec3,
  geom: int,
  flexid: int,
  elemid: int,
  vertid: int,
  worldid: int,
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
  cand_type_out: wp.array[int],
  cand_geomcollisionid_out: wp.array[int],
  ncand_out: wp.array[int],
):
  if dist >= MJ_MAXVAL:
    return

  candid = wp.atomic_add(ncand_out, 0, 1)
  if candid >= max_candidates:
    wp.printf(
      "flex candidate overflow - please increase naconmax to %u\n",
      candid + 1,
    )
    wp.atomic_or(overflow_out, worldid, wp.static(OverflowType.BROADPHASE))
    return

  cand_dist_out[candid] = dist
  cand_pos_out[candid] = pos
  cand_nrm_out[candid] = nrm
  if geom >= 0:
    cand_geom_out[candid] = wp.vec2i(geom, -1)
    cand_flex_out[candid] = wp.vec2i(-1, flexid)
    cand_elem_out[candid] = wp.vec2i(-1, elemid)
    cand_vert_out[candid] = wp.vec2i(-1, vertid)
  elif geom == -2:
    cand_geom_out[candid] = wp.vec2i(-1, -1)
    cand_flex_out[candid] = wp.vec2i(flexid, flexid)
    cand_elem_out[candid] = wp.vec2i(elemid, vertid)
    cand_vert_out[candid] = wp.vec2i(-1, -1)
  else:
    cand_geom_out[candid] = wp.vec2i(-1, -1)
    cand_flex_out[candid] = wp.vec2i(flexid, flexid)
    cand_elem_out[candid] = wp.vec2i(-1, elemid)
    cand_vert_out[candid] = wp.vec2i(vertid, -1)
  cand_worldid_out[candid] = worldid
  cand_type_out[candid] = 1
  cand_geomcollisionid_out[candid] = 0


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
  cand_type_out: wp.array[int],
  cand_geomcollisionid_out: wp.array[int],
  ncand_out: wp.array[int],
):
  if gtype == int(GeomType.SPHERE):
    sphere_radius = size_val[0]
    dist, contact_pos, nrm = collision_primitive_core.sphere_triangle(pos, sphere_radius, t1, t2, t3, tri_radius)
    if dist < margin:
      _write_candidate_contact(
        max_candidates,
        dist,
        contact_pos,
        nrm,
        geomid,
        flexid,
        elemid,
        vertex_id,
        worldid,
        overflow_out,
        cand_dist_out,
        cand_pos_out,
        cand_nrm_out,
        cand_geom_out,
        cand_flex_out,
        cand_elem_out,
        cand_vert_out,
        cand_worldid_out,
        cand_type_out,
        cand_geomcollisionid_out,
        ncand_out,
      )
    return

  # Capsule, box, cylinder all return up to 2 contacts - compute then share writing code
  dists = wp.vec2(collision_primitive_core.MJ_MAXVAL, collision_primitive_core.MJ_MAXVAL)
  poss = collision_primitive_core.mat23f(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
  nrms = collision_primitive_core.mat23f(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

  if gtype == int(GeomType.CAPSULE):
    cap_radius = size_val[0]
    cap_half_len = size_val[1]
    cap_axis = wp.vec3(rot[0, 2], rot[1, 2], rot[2, 2])
    dists, poss, nrms = collision_primitive_core.capsule_triangle(
      pos, cap_axis, cap_radius, cap_half_len, t1, t2, t3, tri_radius
    )
  elif gtype == int(GeomType.BOX):
    dists, poss, nrms = collision_primitive_core.box_triangle(pos, rot, size_val, t1, t2, t3, tri_radius)
  elif gtype == int(GeomType.CYLINDER):
    cyl_radius = size_val[0]
    cyl_half_height = size_val[1]
    cyl_axis = wp.vec3(rot[0, 2], rot[1, 2], rot[2, 2])
    dists, poss, nrms = collision_primitive_core.cylinder_triangle(
      pos, cyl_axis, cyl_radius, cyl_half_height, t1, t2, t3, tri_radius
    )

  # Write up to 2 contacts (shared code for capsule/box/cylinder)
  if dists[0] < margin:
    p1 = wp.vec3(poss[0, 0], poss[0, 1], poss[0, 2])
    n1 = wp.vec3(nrms[0, 0], nrms[0, 1], nrms[0, 2])
    _write_candidate_contact(
      max_candidates,
      dists[0],
      p1,
      n1,
      geomid,
      flexid,
      elemid,
      vertex_id,
      worldid,
      overflow_out,
      cand_dist_out,
      cand_pos_out,
      cand_nrm_out,
      cand_geom_out,
      cand_flex_out,
      cand_elem_out,
      cand_vert_out,
      cand_worldid_out,
      cand_type_out,
      cand_geomcollisionid_out,
      ncand_out,
    )
  if dists[1] < margin:
    p2 = wp.vec3(poss[1, 0], poss[1, 1], poss[1, 2])
    n2 = wp.vec3(nrms[1, 0], nrms[1, 1], nrms[1, 2])
    _write_candidate_contact(
      max_candidates,
      dists[1],
      p2,
      n2,
      geomid,
      flexid,
      elemid,
      vertex_id,
      worldid,
      overflow_out,
      cand_dist_out,
      cand_pos_out,
      cand_nrm_out,
      cand_geom_out,
      cand_flex_out,
      cand_elem_out,
      cand_vert_out,
      cand_worldid_out,
      cand_type_out,
      cand_geomcollisionid_out,
      ncand_out,
    )


@wp.func
def _collide_mesh_triangle(
  # Model:
  mesh_vertadr: wp.array[int],
  mesh_vertnum: wp.array[int],
  mesh_graphadr: wp.array[int],
  mesh_vert: wp.array[wp.vec3],
  mesh_graph: wp.array[int],
  mesh_pos: wp.array[wp.vec3],
  mesh_polynormal: wp.array[wp.vec3],
  mesh_polyvertadr: wp.array[int],
  mesh_polyvert: wp.array[int],
  mesh_polymapadr: wp.array[int],
  mesh_polymapnum: wp.array[int],
  mesh_polymap: wp.array[int],
  # In:
  max_candidates: int,
  mesh_geom_pos: wp.vec3,
  geom_rot: wp.mat33,
  geom_size_val: wp.vec3,
  t1: wp.vec3,
  t2: wp.vec3,
  t3: wp.vec3,
  tri_radius: float,
  margin: float,
  geomid: int,
  flexid: int,
  elemid: int,
  v0_local: int,
  v1_local: int,
  v2_local: int,
  worldid: int,
  did: int,
  epa_vert: wp.array[wp.vec3],
  epa_vert_index: wp.array[int],
  epa_face: wp.array[int],
  epa_pr: wp.array[wp.vec3],
  epa_norm2: wp.array[float],
  epa_horizon: wp.array[int],
  tolerance: float,
  ccd_iterations: int,
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
  cand_type_out: wp.array[int],
  cand_geomcollisionid_out: wp.array[int],
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

  # Construct Triangle Geom (geom2)
  geom2 = Geom()
  geom2.pos = wp.vec3(0.0, 0.0, 0.0)
  geom2.rot = wp.mat33(t1[0], t1[1], t1[2], t2[0], t2[1], t2[2], t3[0], t3[1], t3[2])
  geom2.margin = 0.0
  geom2.index = -1

  centroid = (t1 + t2 + t3) * wp.static(1.0 / 3.0)
  r_geom = wp.length(geom_size_val)
  d1 = wp.length(t1 - centroid)
  d2 = wp.length(t2 - centroid)
  d3 = wp.length(t3 - centroid)
  r_tri = wp.max(d1, wp.max(d2, d3))

  geom_center = mesh_geom_pos
  if did >= 0:
    geom_center = mesh_geom_pos + geom_rot @ mesh_pos[did]

  if wp.length(centroid - geom_center) <= r_geom + r_tri + margin + tri_radius + 0.04:
    dist, ncontact, w1, w2, idx = ccd(
      tolerance,
      margin + tri_radius,
      ccd_iterations,
      ccd_iterations,
      geom1,
      geom2,
      int(GeomType.MESH),
      int(GeomType.TRIANGLE),
      mesh_geom_pos,
      centroid,
      epa_vert,
      epa_vert_index,
      epa_face,
      epa_pr,
      epa_norm2,
      epa_horizon,
    )

    if ncontact > 0 and dist < margin + tri_radius:
      if not _inside_triangle(w2, t1, t2, t3, 0.2):
        return

      if dist < 0.0:
        gjk_normal = wp.normalize(w1 - w2)
      else:
        gjk_normal = wp.normalize(w2 - w1)

      # TODO(thowell): remove after resolving contact normal issue
      best_normal = gjk_normal
      if idx >= 0:
        # Extract GJK/EPA support vertex on the mesh
        f_verts = wp.vec3i(epa_face[idx] & 0x3FF, (epa_face[idx] >> 10) & 0x3FF, (epa_face[idx] >> 20) & 0x3FF)
        sv0 = epa_vert_index[2 * f_verts[0]]

        # Transform w1 to mesh local frame
        w1_local = wp.transpose(geom_rot) @ (w1 - mesh_geom_pos)

        min_plane_dist = float(1e10)
        best_normal_local = wp.transpose(geom_rot) @ gjk_normal
        best_poly_idx = int(-1)

        v_offset = mesh_vertadr[did]
        v_global_idx = v_offset + sv0
        polymap_start = mesh_polymapadr[v_global_idx]
        npolygons = mesh_polymapnum[v_global_idx]

        for k in range(npolygons):
          poly_idx = mesh_polymap[polymap_start + k]
          normal_local = mesh_polynormal[poly_idx]

          v0_local_idx = mesh_polyvert[mesh_polyvertadr[poly_idx]]
          v0_mesh_local = mesh_vert[v_offset + v0_local_idx]

          dist_to_plane = wp.abs(wp.dot(w1_local - v0_mesh_local, normal_local))
          if dist_to_plane < min_plane_dist:
            min_plane_dist = dist_to_plane
            best_normal_local = normal_local
            best_poly_idx = poly_idx

        if best_poly_idx >= 0:
          vert_start = mesh_polyvertadr[best_poly_idx]
          v0_idx = mesh_polyvert[vert_start]
          v1_idx = mesh_polyvert[vert_start + 1]
          v2_idx = mesh_polyvert[vert_start + 2]

          v0_mesh_local = mesh_vert[v_offset + v0_idx]
          v1_mesh_local = mesh_vert[v_offset + v1_idx]
          v2_mesh_local = mesh_vert[v_offset + v2_idx]

          # Filter out contacts that are too far from the plane or fall outside the face
          if min_plane_dist > 0.005 or not _inside_triangle(w1_local, v0_mesh_local, v1_mesh_local, v2_mesh_local, 0.05):
            return

        best_normal = wp.normalize(geom_rot @ best_normal_local)

      normal = wp.where(wp.dot(best_normal, gjk_normal) >= 0.0, best_normal, -best_normal)
      contact_pos = 0.5 * (w1 + w2)

      dist_v0 = wp.dot(t1 - w1, normal) - tri_radius
      dist_v1 = wp.dot(t2 - w1, normal) - tri_radius
      dist_v2 = wp.dot(t3 - w1, normal) - tri_radius

      min_dist = wp.min(dist_v0, wp.min(dist_v1, dist_v2))
      if min_dist < margin:
        deepest_vert = v0_local
        pos = t1 - normal * (tri_radius + 0.5 * dist_v0)
        if dist_v1 < dist_v0 and dist_v1 < dist_v2:
          deepest_vert = v1_local
          pos = t2 - normal * (tri_radius + 0.5 * dist_v1)
        elif dist_v2 < dist_v0 and dist_v2 < dist_v1:
          deepest_vert = v2_local
          pos = t3 - normal * (tri_radius + 0.5 * dist_v2)

        _write_candidate_contact(
          max_candidates,
          min_dist,
          pos,
          normal,
          geomid,
          flexid,
          elemid,
          -1,
          worldid,
          overflow_out,
          cand_dist_out,
          cand_pos_out,
          cand_nrm_out,
          cand_geom_out,
          cand_flex_out,
          cand_elem_out,
          cand_vert_out,
          cand_worldid_out,
          cand_type_out,
          cand_geomcollisionid_out,
          ncand_out,
        )

  return


@wp.kernel
def _flex_plane_narrowphase(
  # Model:
  ngeom: int,
  nflexvert: int,
  geom_type: wp.array[int],
  geom_condim: wp.array[int],
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
  flex_vertadr: wp.array[int],
  flex_radius: wp.array[float],
  flex_vertflexid: wp.array[int],
  # Data in:
  geom_xpos_in: wp.array2d[wp.vec3],
  geom_xmat_in: wp.array2d[wp.mat33],
  flexvert_xpos_in: wp.array2d[wp.vec3],
  nworld_in: int,
  naconmax_in: int,
  ncollision_in: wp.array[int],
  # In:
  collision_pair_in: wp.array[wp.vec2i],
  collision_worldid_in: wp.array[int],
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
  contact_worldid_out: wp.array[int],
  contact_type_out: wp.array[int],
  contact_geomcollisionid_out: wp.array[int],
  nacon_out: wp.array[int],
):
  collisionid = wp.tid()
  if collisionid >= ncollision_in[0] or collisionid >= collision_pair_in.shape[0]:
    return

  pair = collision_pair_in[collisionid]
  geomid = pair[1]

  gtype = geom_type[geomid]
  if gtype != int(GeomType.PLANE):
    return

  vertid = pair[0]
  worldid = collision_worldid_in[collisionid]

  flexid = flex_vertflexid[vertid]
  radius = flex_radius[flexid]
  flex_margin_val = flex_margin[flexid]
  # Convert global vertid to local vertex index within this flex
  local_vertid = vertid - flex_vertadr[flexid]

  vert = flexvert_xpos_in[worldid, vertid]

  plane_pos = geom_xpos_in[worldid, geomid]
  plane_rot = geom_xmat_in[worldid, geomid]
  plane_normal = wp.vec3(plane_rot[0, 2], plane_rot[1, 2], plane_rot[2, 2])

  margin = geom_margin[worldid % geom_margin.shape[0], geomid] + flex_margin_val

  diff = vert - plane_pos
  signed_dist = wp.dot(diff, plane_normal)
  dist = signed_dist - radius

  if dist < margin:
    contact_pos = vert - plane_normal * (dist * 0.5 + radius)
    _write_flex_contact(
      geom_condim,
      geom_priority,
      geom_solmix,
      geom_solref,
      geom_solimp,
      geom_friction,
      geom_gap,
      flex_condim,
      flex_priority,
      flex_solmix,
      flex_solref,
      flex_solimp,
      flex_friction,
      flex_margin,
      flex_gap,
      naconmax_in,
      collisionid,
      dist,
      contact_pos,
      plane_normal,
      margin,
      geomid,
      flexid,
      -1,
      local_vertid,
      worldid,
      contact_dist_out,
      contact_pos_out,
      contact_frame_out,
      contact_includemargin_out,
      contact_friction_out,
      contact_solref_out,
      contact_solreffriction_out,
      contact_solimp_out,
      contact_dim_out,
      contact_geom_out,
      contact_flex_out,
      contact_elem_out,
      contact_vert_out,
      contact_worldid_out,
      contact_type_out,
      contact_geomcollisionid_out,
      nacon_out,
    )


@wp.kernel
def _flex_geom_vertex_narrowphase_detect(
  # Model:
  ngeom: int,
  nflexvert: int,
  geom_type: wp.array[int],
  geom_contype: wp.array[int],
  geom_conaffinity: wp.array[int],
  geom_size: wp.array2d[wp.vec3],
  geom_margin: wp.array2d[float],
  flex_contype: wp.array[int],
  flex_conaffinity: wp.array[int],
  flex_margin: wp.array[float],
  flex_dim: wp.array[int],
  flex_vertadr: wp.array[int],
  flex_radius: wp.array[float],
  flex_vertflexid: wp.array[int],
  # Data in:
  geom_xpos_in: wp.array2d[wp.vec3],
  geom_xmat_in: wp.array2d[wp.mat33],
  flexvert_xpos_in: wp.array2d[wp.vec3],
  nworld_in: int,
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
  cand_type_out: wp.array[int],
  cand_geomcollisionid_out: wp.array[int],
  ncand_out: wp.array[int],
):
  worldid, vertid = wp.tid()

  flexid = flex_vertflexid[vertid]
  if flex_dim[flexid] >= 2:
    return

  radius = flex_radius[flexid]
  flex_margin_val = flex_margin[flexid]
  local_vertid = vertid - flex_vertadr[flexid]

  v_pos = flexvert_xpos_in[worldid, vertid]

  for geomid in range(ngeom):
    gtype = geom_type[geomid]
    if (
      gtype != int(GeomType.SPHERE)
      and gtype != int(GeomType.CAPSULE)
      and gtype != int(GeomType.BOX)
      and gtype != int(GeomType.CYLINDER)
    ):
      continue

    g_contype = geom_contype[geomid]
    g_conaffinity = geom_conaffinity[geomid]
    f_contype = flex_contype[flexid]
    f_conaffinity = flex_conaffinity[flexid]
    if not ((g_contype & f_conaffinity) or (f_contype & g_conaffinity)):
      continue

    geom_margin_val = geom_margin[worldid % geom_margin.shape[0], geomid]
    margin = geom_margin_val + flex_margin_val

    geom_pos = geom_xpos_in[worldid, geomid]
    geom_rot = geom_xmat_in[worldid, geomid]
    geom_size_val = geom_size[worldid % geom_size.shape[0], geomid]

    dist = collision_primitive_core.MJ_MAXVAL
    contact_pos = wp.vec3(0.0)
    nrm = wp.vec3(0.0)

    if gtype == int(GeomType.SPHERE):
      sphere_radius = geom_size_val[0]
      dist, contact_pos, nrm = collision_primitive_core.sphere_sphere(v_pos, radius, geom_pos, sphere_radius)
    elif gtype == int(GeomType.CAPSULE):
      cap_radius = geom_size_val[0]
      cap_half_len = geom_size_val[1]
      cap_axis = wp.vec3(geom_rot[0, 2], geom_rot[1, 2], geom_rot[2, 2])
      dist, contact_pos, nrm = collision_primitive_core.sphere_capsule(
        v_pos, radius, geom_pos, cap_axis, cap_radius, cap_half_len
      )
    elif gtype == int(GeomType.BOX):
      dist, contact_pos, nrm = collision_primitive_core.sphere_box(v_pos, radius, geom_pos, geom_rot, geom_size_val)
    elif gtype == int(GeomType.CYLINDER):
      cyl_radius = geom_size_val[0]
      cyl_half_height = geom_size_val[1]
      cyl_axis = wp.vec3(geom_rot[0, 2], geom_rot[1, 2], geom_rot[2, 2])
      dist, contact_pos, nrm = collision_primitive_core.sphere_cylinder(
        v_pos, radius, geom_pos, cyl_axis, cyl_radius, cyl_half_height
      )

    if dist < margin:
      _write_candidate_contact(
        max_candidates,
        dist,
        contact_pos,
        nrm,
        geomid,
        flexid,
        -1,
        local_vertid,
        worldid,
        overflow_out,
        cand_dist_out,
        cand_pos_out,
        cand_nrm_out,
        cand_geom_out,
        cand_flex_out,
        cand_elem_out,
        cand_vert_out,
        cand_worldid_out,
        cand_type_out,
        cand_geomcollisionid_out,
        ncand_out,
      )


@wp.func
def _sphere_tetrahedron(
  # In:
  sphere_pos: wp.vec3,
  sphere_radius: float,
  t0: wp.vec3,
  t1: wp.vec3,
  t2: wp.vec3,
  t3: wp.vec3,
  tri_radius: float,
) -> Tuple[float, wp.vec3, wp.vec3]:
  d0, p0, n0 = collision_primitive_core.sphere_triangle(sphere_pos, sphere_radius, t0, t1, t2, tri_radius)
  d1, p1, n1 = collision_primitive_core.sphere_triangle(sphere_pos, sphere_radius, t0, t2, t3, tri_radius)
  d2, p2, n2 = collision_primitive_core.sphere_triangle(sphere_pos, sphere_radius, t0, t3, t1, tri_radius)
  d3, p3, n3 = collision_primitive_core.sphere_triangle(sphere_pos, sphere_radius, t1, t3, t2, tri_radius)

  min_d = d0
  min_p = p0
  min_n = n0

  if d1 < min_d:
    min_d = d1
    min_p = p1
    min_n = n1
  if d2 < min_d:
    min_d = d2
    min_p = p2
    min_n = n2
  if d3 < min_d:
    min_d = d3
    min_p = p3
    min_n = n3

  return min_d, min_p, min_n


@wp.func
def _plane_vertex(
  # In:
  pos_v: wp.vec3,
  rad: float,
  t0: wp.vec3,
  t1: wp.vec3,
  t2: wp.vec3,
) -> Tuple[bool, float, wp.vec3, wp.vec3]:
  e1 = t1 - t0
  e2 = t2 - t0
  ev = pos_v - t0

  nrm = wp.normalize(wp.cross(e1, e2))
  dst = wp.dot(ev, nrm)
  if dst >= 0.0 or dst <= -2.0 * rad:
    return False, 0.0, wp.vec3(0.0), wp.vec3(0.0)

  dist = -dst - 2.0 * rad
  nrm_out = -nrm
  contact_pos = pos_v - nrm * (0.5 * dst)
  return True, dist, contact_pos, nrm_out


@wp.kernel(module="unique", enable_backward=False)
def _flex_internal_collisions_detect(
  # Model:
  nflex: int,
  flex_margin: wp.array[float],
  flex_internal: wp.array[int],
  flex_dim: wp.array[int],
  flex_vertadr: wp.array[int],
  flex_elemdataadr: wp.array[int],
  flex_evpairadr: wp.array[int],
  flex_evpairnum: wp.array[int],
  flex_elem: wp.array[int],
  flex_evpair: wp.array[wp.vec2i],
  flex_radius: wp.array[float],
  flex_evpairflexid: wp.array[int],
  # Data in:
  flexvert_xpos_in: wp.array2d[wp.vec3],
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
  cand_type_out: wp.array[int],
  cand_geomcollisionid_out: wp.array[int],
  ncand_out: wp.array[int],
):
  worldid, pair_idx = wp.tid()

  flexid = flex_evpairflexid[pair_idx]
  if flex_internal[flexid] == 0:
    return

  ev = flex_evpair[pair_idx]
  e = ev[0]
  v = ev[1]

  dim = flex_dim[flexid]
  radius = flex_radius[flexid]
  margin = flex_margin[flexid]
  vert_adr = flex_vertadr[flexid]

  sphere_pos = flexvert_xpos_in[worldid, vert_adr + v]

  elem_data_idx = flex_elemdataadr[flexid] + e * (dim + 1)
  v0_local = flex_elem[elem_data_idx]
  p0 = flexvert_xpos_in[worldid, vert_adr + v0_local]

  dist = float(MJ_MAXVAL)
  contact_pos = wp.vec3(0.0)
  nrm = wp.vec3(0.0)

  if dim == 1:
    v1_local = flex_elem[elem_data_idx + 1]
    p1 = flexvert_xpos_in[worldid, vert_adr + v1_local]
    capsule_pos = 0.5 * (p0 + p1)
    capsule_axis = wp.normalize(p1 - p0)
    capsule_half_len = 0.5 * wp.length(p1 - p0)
    dist, contact_pos, nrm = collision_primitive_core.sphere_capsule(
      sphere_pos, radius, capsule_pos, capsule_axis, radius, capsule_half_len
    )
  elif dim == 2:
    v1_local = flex_elem[elem_data_idx + 1]
    v2_local = flex_elem[elem_data_idx + 2]
    p1 = flexvert_xpos_in[worldid, vert_adr + v1_local]
    p2 = flexvert_xpos_in[worldid, vert_adr + v2_local]
    dist, contact_pos, nrm = collision_primitive_core.sphere_triangle(sphere_pos, radius, p0, p1, p2, radius)
  elif dim == 3:
    v1_local = flex_elem[elem_data_idx + 1]
    v2_local = flex_elem[elem_data_idx + 2]
    v3_local = flex_elem[elem_data_idx + 3]
    p1 = flexvert_xpos_in[worldid, vert_adr + v1_local]
    p2 = flexvert_xpos_in[worldid, vert_adr + v2_local]
    p3 = flexvert_xpos_in[worldid, vert_adr + v3_local]
    dist, contact_pos, nrm = _sphere_tetrahedron(sphere_pos, radius, p0, p1, p2, p3, radius)

  if dist < margin:
    _write_candidate_contact(
      max_candidates,
      dist,
      contact_pos,
      nrm,
      -1,
      flexid,
      e,
      v,
      worldid,
      overflow_out,
      cand_dist_out,
      cand_pos_out,
      cand_nrm_out,
      cand_geom_out,
      cand_flex_out,
      cand_elem_out,
      cand_vert_out,
      cand_worldid_out,
      cand_type_out,
      cand_geomcollisionid_out,
      ncand_out,
    )


@wp.kernel(module="unique", enable_backward=False)
def _flex_tet_internal_collisions_detect(
  # Model:
  nflex: int,
  flex_dim: wp.array[int],
  flex_vertadr: wp.array[int],
  flex_elemadr: wp.array[int],
  flex_elemnum: wp.array[int],
  flex_elemdataadr: wp.array[int],
  flex_elem: wp.array[int],
  flex_radius: wp.array[float],
  flex_elemflexid: wp.array[int],
  # Data in:
  flexvert_xpos_in: wp.array2d[wp.vec3],
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
  cand_type_out: wp.array[int],
  cand_geomcollisionid_out: wp.array[int],
  ncand_out: wp.array[int],
):
  worldid, elemid = wp.tid()

  flexid = flex_elemflexid[elemid]
  if flex_dim[flexid] != 3:
    return

  radius = flex_radius[flexid]
  vert_adr = flex_vertadr[flexid]

  local_elemid = elemid - flex_elemadr[flexid]
  elem_data_idx = flex_elemdataadr[flexid] + local_elemid * 4

  v0 = flex_elem[elem_data_idx]
  v1 = flex_elem[elem_data_idx + 1]
  v2 = flex_elem[elem_data_idx + 2]
  v3 = flex_elem[elem_data_idx + 3]

  p0 = flexvert_xpos_in[worldid, vert_adr + v0]
  p1 = flexvert_xpos_in[worldid, vert_adr + v1]
  p2 = flexvert_xpos_in[worldid, vert_adr + v2]
  p3 = flexvert_xpos_in[worldid, vert_adr + v3]

  # Test face (0,1,2) vs Vertex 3
  ok0, dist0, pos0, nrm0 = _plane_vertex(p3, radius, p0, p1, p2)
  if ok0:
    _write_candidate_contact(
      max_candidates,
      dist0,
      pos0,
      nrm0,
      -1,
      flexid,
      local_elemid,
      v3,
      worldid,
      overflow_out,
      cand_dist_out,
      cand_pos_out,
      cand_nrm_out,
      cand_geom_out,
      cand_flex_out,
      cand_elem_out,
      cand_vert_out,
      cand_worldid_out,
      cand_type_out,
      cand_geomcollisionid_out,
      ncand_out,
    )

  # Test face (0,2,3) vs Vertex 1
  ok1, dist1, pos1, nrm1 = _plane_vertex(p1, radius, p0, p2, p3)
  if ok1:
    _write_candidate_contact(
      max_candidates,
      dist1,
      pos1,
      nrm1,
      -1,
      flexid,
      local_elemid,
      v1,
      worldid,
      overflow_out,
      cand_dist_out,
      cand_pos_out,
      cand_nrm_out,
      cand_geom_out,
      cand_flex_out,
      cand_elem_out,
      cand_vert_out,
      cand_worldid_out,
      cand_type_out,
      cand_geomcollisionid_out,
      ncand_out,
    )

  # Test face (0,3,1) vs Vertex 2
  ok2, dist2, pos2, nrm2 = _plane_vertex(p2, radius, p0, p3, p1)
  if ok2:
    _write_candidate_contact(
      max_candidates,
      dist2,
      pos2,
      nrm2,
      -1,
      flexid,
      local_elemid,
      v2,
      worldid,
      overflow_out,
      cand_dist_out,
      cand_pos_out,
      cand_nrm_out,
      cand_geom_out,
      cand_flex_out,
      cand_elem_out,
      cand_vert_out,
      cand_worldid_out,
      cand_type_out,
      cand_geomcollisionid_out,
      ncand_out,
    )

  # Test face (1,3,2) vs Vertex 0
  ok3, dist3, pos3, nrm3 = _plane_vertex(p0, radius, p1, p3, p2)
  if ok3:
    _write_candidate_contact(
      max_candidates,
      dist3,
      pos3,
      nrm3,
      -1,
      flexid,
      local_elemid,
      v0,
      worldid,
      overflow_out,
      cand_dist_out,
      cand_pos_out,
      cand_nrm_out,
      cand_geom_out,
      cand_flex_out,
      cand_elem_out,
      cand_vert_out,
      cand_worldid_out,
      cand_type_out,
      cand_geomcollisionid_out,
      ncand_out,
    )


@wp.func
def _inside_triangle(
  # In:
  P: wp.vec3,
  A: wp.vec3,
  B: wp.vec3,
  C: wp.vec3,
  tol: float,
) -> bool:
  v0 = B - A
  v1 = C - A
  v2 = P - A
  d00 = wp.dot(v0, v0)
  d01 = wp.dot(v0, v1)
  d11 = wp.dot(v1, v1)
  d20 = wp.dot(v2, v0)
  d21 = wp.dot(v2, v1)
  denom = d00 * d11 - d01 * d01
  if wp.abs(denom) < 1e-12:
    return False
  v = (d11 * d20 - d01 * d21) / denom
  w = (d00 * d21 - d01 * d20) / denom
  u = 1.0 - v - w
  return u >= -tol and v >= -tol and w >= -tol and u <= 1.0 + tol and v <= 1.0 + tol and w <= 1.0 + tol


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
def _elements_overlap(
  # Data in:
  flexvert_xpos_in: wp.array2d[wp.vec3],
  # In:
  dim: int,
  radius: float,
  v1_indices: wp.vec4i,
  v2_indices: wp.vec4i,
  vert_adr: int,
  worldid: int,
) -> bool:
  p1_0 = flexvert_xpos_in[worldid, vert_adr + v1_indices[0]]
  p1_1 = flexvert_xpos_in[worldid, vert_adr + v1_indices[1]]

  min1 = wp.min(p1_0, p1_1)
  max1 = wp.max(p1_0, p1_1)

  if dim >= 2:
    p1_2 = flexvert_xpos_in[worldid, vert_adr + v1_indices[2]]
    min1 = wp.min(min1, p1_2)
    max1 = wp.max(max1, p1_2)
  if dim >= 3:
    p1_3 = flexvert_xpos_in[worldid, vert_adr + v1_indices[3]]
    min1 = wp.min(min1, p1_3)
    max1 = wp.max(max1, p1_3)

  p2_0 = flexvert_xpos_in[worldid, vert_adr + v2_indices[0]]
  p2_1 = flexvert_xpos_in[worldid, vert_adr + v2_indices[1]]

  min2 = wp.min(p2_0, p2_1)
  max2 = wp.max(p2_0, p2_1)

  if dim >= 2:
    p2_2 = flexvert_xpos_in[worldid, vert_adr + v2_indices[2]]
    min2 = wp.min(min2, p2_2)
    max2 = wp.max(max2, p2_2)
  if dim >= 3:
    p2_3 = flexvert_xpos_in[worldid, vert_adr + v2_indices[3]]
    min2 = wp.min(min2, p2_3)
    max2 = wp.max(max2, p2_3)

  rbound = 2.0 * radius

  if min1[0] - rbound > max2[0] or max1[0] + rbound < min2[0]:
    return False
  if min1[1] - rbound > max2[1] or max1[1] + rbound < min2[1]:
    return False
  if min1[2] - rbound > max2[2] or max1[2] + rbound < min2[2]:
    return False

  return True


@wp.kernel
def _flex_sap_project(
  # Model:
  nflex: int,
  flex_selfcollide: wp.array[int],
  flex_dim: wp.array[int],
  flex_vertadr: wp.array[int],
  flex_elemadr: wp.array[int],
  flex_elemdataadr: wp.array[int],
  flex_elem: wp.array[int],
  flex_radius: wp.array[float],
  flex_elemflexid: wp.array[int],
  # Data in:
  flexvert_xpos_in: wp.array2d[wp.vec3],
  nworld_in: int,
  # In:
  nelem: int,
  direction: wp.vec3,
  # Out:
  projection_lower_out: wp.array2d[float],
  projection_upper_out: wp.array2d[float],
  sort_index_out: wp.array2d[int],
  elem_aabb_lower_out: wp.array2d[wp.vec3],
  elem_aabb_upper_out: wp.array2d[wp.vec3],
  segmented_index_out: wp.array[int],
):
  worldid, elemid = wp.tid()

  flexid = flex_elemflexid[elemid]

  # Initialize sort index
  sort_index_out[worldid, elemid] = elemid

  # Compute AABB from vertex positions
  dim = flex_dim[flexid]
  vert_adr = flex_vertadr[flexid]
  elem_adr = flex_elemadr[flexid]
  e = elemid - elem_adr
  elem_data_idx = flex_elemdataadr[flexid] + e * (dim + 1)

  v = _get_element_vertices(flex_elem, dim, elem_data_idx)

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
  rbound = 2.0 * radius
  inflate = wp.vec3(rbound, rbound, rbound)
  aabb_min = aabb_min - inflate
  aabb_max = aabb_max + inflate

  elem_aabb_lower_out[worldid, elemid] = aabb_min
  elem_aabb_upper_out[worldid, elemid] = aabb_max

  # Project AABB onto direction to get 1D interval
  center = 0.5 * (aabb_min + aabb_max)
  halfsize = 0.5 * (aabb_max - aabb_min)
  proj_center = wp.dot(direction, center)
  proj_radius = wp.abs(direction[0]) * halfsize[0] + wp.abs(direction[1]) * halfsize[1] + wp.abs(direction[2]) * halfsize[2]

  # If self-collision is disabled for this flex, push to infinity
  if flex_selfcollide[flexid] == 0:
    projection_lower_out[worldid, elemid] = MJ_MAXVAL
    projection_upper_out[worldid, elemid] = MJ_MAXVAL
  else:
    projection_lower_out[worldid, elemid] = proj_center - proj_radius
    projection_upper_out[worldid, elemid] = proj_center + proj_radius

  # Segmented sort boundaries
  if elemid == 0:
    segmented_index_out[worldid] = worldid * nelem
    if worldid == nworld_in - 1:
      segmented_index_out[nworld_in] = nworld_in * nelem


@wp.kernel
def _flex_sap_filter(
  # Model:
  flex_selfcollide: wp.array[int],
  flex_dim: wp.array[int],
  flex_vertadr: wp.array[int],
  flex_elemadr: wp.array[int],
  flex_elemnum: wp.array[int],
  flex_elemdataadr: wp.array[int],
  flex_vertbodyid: wp.array[int],
  flex_elem: wp.array[int],
  flex_elemflexid: wp.array[int],
  # In:
  raw_npairs_in: wp.array[int],
  raw_pair_elem1_in: wp.array[int],
  raw_pair_elem2_in: wp.array[int],
  raw_pair_worldid_in: wp.array[int],
  # Out:
  npairs_out: wp.array[int],
  pair_elem1_out: wp.array[int],
  pair_elem2_out: wp.array[int],
  pair_worldid_out: wp.array[int],
):
  """Filter raw SAP pairs for flex self-collision."""
  tid = wp.tid()

  # Skip if beyond actual pair count
  n_raw = raw_npairs_in[0]
  if tid >= n_raw:
    return

  elem1_global = raw_pair_elem1_in[tid]
  elem2_global = raw_pair_elem2_in[tid]

  # Both elements must belong to the same flex
  flexid1 = flex_elemflexid[elem1_global]
  flexid2 = flex_elemflexid[elem2_global]
  if flexid1 != flexid2:
    return

  flexid = flexid1
  if flex_selfcollide[flexid] == 0:
    return

  # Exclude elements sharing vertices/bodies
  dim = flex_dim[flexid]
  vert_adr = flex_vertadr[flexid]
  elem_adr = flex_elemadr[flexid]

  e1 = elem1_global - elem_adr
  e2 = elem2_global - elem_adr
  elem_data_idx1 = flex_elemdataadr[flexid] + e1 * (dim + 1)
  elem_data_idx2 = flex_elemdataadr[flexid] + e2 * (dim + 1)
  v1_indices = _get_element_vertices(flex_elem, dim, elem_data_idx1)
  v2_indices = _get_element_vertices(flex_elem, dim, elem_data_idx2)

  if _exclude_self_collision(flex_vertbodyid, v1_indices, dim + 1, v2_indices, dim + 1, vert_adr):
    return

  # Output this pair
  idx = wp.atomic_add(npairs_out, 0, 1)
  if idx < pair_elem1_out.shape[0]:
    pair_elem1_out[idx] = elem1_global
    pair_elem2_out[idx] = elem2_global
    pair_worldid_out[idx] = raw_pair_worldid_in[tid]


@wp.kernel
def _flex_selfcollision_narrowphase(
  # Model:
  nflex: int,
  opt_ccd_tolerance: wp.array[float],
  flex_dim: wp.array[int],
  flex_vertadr: wp.array[int],
  flex_elemadr: wp.array[int],
  flex_elemdataadr: wp.array[int],
  flex_elem: wp.array[int],
  flex_radius: wp.array[float],
  flex_elemflexid: wp.array[int],
  # Data in:
  flexvert_xpos_in: wp.array2d[wp.vec3],
  # In:
  max_candidates: int,
  gjk_iterations: int,
  epa_iterations: int,
  npairs_in: wp.array[int],
  pair_elem1_in: wp.array[int],
  pair_elem2_in: wp.array[int],
  pair_worldid_in: wp.array[int],
  max_pairs: int,
  n_total_elems: int,
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
  cand_type_out: wp.array[int],
  cand_geomcollisionid_out: wp.array[int],
  ncand_out: wp.array[int],
):
  """Process SAP-identified pairs through narrowphase (GJK/EPA)."""
  pairid = wp.tid()

  # Check bounds
  actual_npairs = npairs_in[0]
  if pairid >= actual_npairs or pairid >= max_pairs:
    return

  elem1_global = pair_elem1_in[pairid]
  elem2_global = pair_elem2_in[pairid]
  worldid = pair_worldid_in[pairid]

  flexid = flex_elemflexid[elem1_global]
  radius = flex_radius[flexid]
  dim = flex_dim[flexid]
  vert_adr = flex_vertadr[flexid]
  elem_adr = flex_elemadr[flexid]

  e1 = elem1_global - elem_adr
  e2 = elem2_global - elem_adr
  elem_data_idx1 = flex_elemdataadr[flexid] + e1 * (dim + 1)
  elem_data_idx2 = flex_elemdataadr[flexid] + e2 * (dim + 1)

  v1_indices = _get_element_vertices(flex_elem, dim, elem_data_idx1)
  v2_indices = _get_element_vertices(flex_elem, dim, elem_data_idx2)

  # Workspace for this pair
  offset1 = pairid * 8
  for idx in range(dim + 1):
    workspace_verts_out[offset1 + idx] = flexvert_xpos_in[worldid, vert_adr + v1_indices[idx]]

  if dim == 1:
    # Capsule-capsule collision
    p0 = workspace_verts_out[offset1]
    p1 = workspace_verts_out[offset1 + 1]
    cap1_pos = 0.5 * (p0 + p1)
    cap1_axis = wp.normalize(p1 - p0)
    cap1_half_len = 0.5 * wp.length(p1 - p0)

    p2_0 = flexvert_xpos_in[worldid, vert_adr + v2_indices[0]]
    p2_1 = flexvert_xpos_in[worldid, vert_adr + v2_indices[1]]
    cap2_pos = 0.5 * (p2_0 + p2_1)
    cap2_axis = wp.normalize(p2_1 - p2_0)
    cap2_half_len = 0.5 * wp.length(p2_1 - p2_0)

    margin = 0.0

    contact_dist, contact_pos, contact_normal = collision_primitive_core.capsule_capsule(
      cap1_pos, cap1_axis, radius, cap1_half_len, cap2_pos, cap2_axis, radius, cap2_half_len, margin
    )

    for c in range(2):
      d_val = contact_dist[c]
      if d_val < 0.0:
        _write_candidate_contact(
          max_candidates,
          d_val,
          contact_pos[c],
          contact_normal[c],
          -2,
          flexid,
          e1,
          e2,
          worldid,
          overflow_out,
          cand_dist_out,
          cand_pos_out,
          cand_nrm_out,
          cand_geom_out,
          cand_flex_out,
          cand_elem_out,
          cand_vert_out,
          cand_worldid_out,
          cand_type_out,
          cand_geomcollisionid_out,
          ncand_out,
        )
  else:
    # GJK/EPA for dim >= 2
    offset2 = pairid * 8 + 4
    for idx in range(dim + 1):
      workspace_verts_out[offset2 + idx] = flexvert_xpos_in[worldid, vert_adr + v2_indices[idx]]

    geom1 = Geom()
    geom1.pos = wp.vec3(0.0)
    geom1.rot = wp.identity(n=3, dtype=float)
    geom1.size = wp.vec3(0.0)
    geom1.margin = 2.0 * radius
    geom1.vert = workspace_verts_out
    geom1.vertadr = offset1
    geom1.vertnum = dim + 1
    geom1.graphadr = -1
    geom1.index = -1

    geom2 = Geom()
    geom2.pos = wp.vec3(0.0)
    geom2.rot = wp.identity(n=3, dtype=float)
    geom2.size = wp.vec3(0.0)
    geom2.margin = 2.0 * radius
    geom2.vert = workspace_verts_out
    geom2.vertadr = offset2
    geom2.vertnum = dim + 1
    geom2.graphadr = -1
    geom2.index = -1

    center1 = wp.vec3(0.0)
    for idx in range(dim + 1):
      center1 += workspace_verts_out[offset1 + idx]
    center1 = center1 / float(dim + 1)

    center2 = wp.vec3(0.0)
    for idx in range(dim + 1):
      center2 += workspace_verts_out[offset2 + idx]
    center2 = center2 / float(dim + 1)

    tol = opt_ccd_tolerance[0 % opt_ccd_tolerance.shape[0]]

    dist, ncontact, w1, w2, _ = ccd(
      tol,
      2.0 * radius,
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
    )

    phys_dist = dist
    if ncontact > 0 and phys_dist < 0.0:
      p1_0 = workspace_verts_out[offset1]
      p1_1 = workspace_verts_out[offset1 + 1]
      p1_2 = workspace_verts_out[offset1 + 2]
      p2_0 = workspace_verts_out[offset2]
      p2_1 = workspace_verts_out[offset2 + 1]
      p2_2 = workspace_verts_out[offset2 + 2]
      if not (_inside_triangle(w1, p1_0, p1_1, p1_2, 0.2) and _inside_triangle(w2, p2_0, p2_1, p2_2, 0.2)):
        return

      pos = 0.5 * (w1 + w2)
      nrm = wp.normalize(w1 - w2)
      _write_candidate_contact(
        max_candidates,
        phys_dist,
        pos,
        nrm,
        -2,
        flexid,
        e1,
        e2,
        worldid,
        overflow_out,
        cand_dist_out,
        cand_pos_out,
        cand_nrm_out,
        cand_geom_out,
        cand_flex_out,
        cand_elem_out,
        cand_vert_out,
        cand_worldid_out,
        cand_type_out,
        cand_geomcollisionid_out,
        ncand_out,
      )


@wp.kernel(module="unique", enable_backward=False)
def _flex_active_element_collisions_detect(
  # Model:
  nflex: int,
  opt_ccd_tolerance: wp.array[float],
  flex_selfcollide: wp.array[int],
  flex_dim: wp.array[int],
  flex_vertadr: wp.array[int],
  flex_elemadr: wp.array[int],
  flex_elemnum: wp.array[int],
  flex_elemdataadr: wp.array[int],
  flex_vertbodyid: wp.array[int],
  flex_elem: wp.array[int],
  flex_radius: wp.array[float],
  flex_elemflexid: wp.array[int],
  # Data in:
  flexvert_xpos_in: wp.array2d[wp.vec3],
  # In:
  max_candidates: int,
  gjk_iterations: int,
  epa_iterations: int,
  n_total_elems: int,
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
  cand_type_out: wp.array[int],
  cand_geomcollisionid_out: wp.array[int],
  ncand_out: wp.array[int],
):
  worldid, elem1_global = wp.tid()

  flexid = flex_elemflexid[elem1_global]
  if flex_selfcollide[flexid] == 0:
    return

  radius = flex_radius[flexid]
  dim = flex_dim[flexid]
  vert_adr = flex_vertadr[flexid]
  elem_adr = flex_elemadr[flexid]
  elem_num = flex_elemnum[flexid]

  e1 = elem1_global - elem_adr
  elem_data_idx1 = flex_elemdataadr[flexid] + e1 * (dim + 1)

  v1_indices = _get_element_vertices(flex_elem, dim, elem_data_idx1)

  unique_thread_id = worldid * n_total_elems + elem1_global

  offset1 = unique_thread_id * 8
  for idx in range(dim + 1):
    workspace_verts_out[offset1 + idx] = flexvert_xpos_in[worldid, vert_adr + v1_indices[idx]]

  for e2 in range(e1 + 1, elem_num):
    elem_data_idx2 = flex_elemdataadr[flexid] + e2 * (dim + 1)
    v2_indices = _get_element_vertices(flex_elem, dim, elem_data_idx2)

    if _exclude_self_collision(flex_vertbodyid, v1_indices, dim + 1, v2_indices, dim + 1, vert_adr):
      continue

    overlap = _elements_overlap(flexvert_xpos_in, dim, radius, v1_indices, v2_indices, vert_adr, worldid)
    if not overlap:
      continue

    if dim == 1:
      p0 = workspace_verts_out[offset1]
      p1 = workspace_verts_out[offset1 + 1]
      cap1_pos = 0.5 * (p0 + p1)
      cap1_axis = wp.normalize(p1 - p0)
      cap1_half_len = 0.5 * wp.length(p1 - p0)

      p2_0 = flexvert_xpos_in[worldid, vert_adr + v2_indices[0]]
      p2_1 = flexvert_xpos_in[worldid, vert_adr + v2_indices[1]]
      cap2_pos = 0.5 * (p2_0 + p2_1)
      cap2_axis = wp.normalize(p2_1 - p2_0)
      cap2_half_len = 0.5 * wp.length(p2_1 - p2_0)

      margin = 0.0

      contact_dist, contact_pos, contact_normal = collision_primitive_core.capsule_capsule(
        cap1_pos, cap1_axis, radius, cap1_half_len, cap2_pos, cap2_axis, radius, cap2_half_len, margin
      )

      for c in range(2):
        d_val = contact_dist[c]
        if d_val < 0.0:
          _write_candidate_contact(
            max_candidates,
            d_val,
            contact_pos[c],
            contact_normal[c],
            -2,
            flexid,
            e1,
            e2,
            worldid,
            overflow_out,
            cand_dist_out,
            cand_pos_out,
            cand_nrm_out,
            cand_geom_out,
            cand_flex_out,
            cand_elem_out,
            cand_vert_out,
            cand_worldid_out,
            cand_type_out,
            cand_geomcollisionid_out,
            ncand_out,
          )
    else:
      offset2 = unique_thread_id * 8 + 4
      for idx in range(dim + 1):
        workspace_verts_out[offset2 + idx] = flexvert_xpos_in[worldid, vert_adr + v2_indices[idx]]

      geom1 = Geom()
      geom1.pos = wp.vec3(0.0)
      geom1.rot = wp.identity(n=3, dtype=float)
      geom1.size = wp.vec3(0.0)
      geom1.margin = 2.0 * radius
      geom1.vert = workspace_verts_out
      geom1.vertadr = offset1
      geom1.vertnum = dim + 1
      geom1.graphadr = -1
      geom1.index = -1

      geom2 = Geom()
      geom2.pos = wp.vec3(0.0)
      geom2.rot = wp.identity(n=3, dtype=float)
      geom2.size = wp.vec3(0.0)
      geom2.margin = 2.0 * radius
      geom2.vert = workspace_verts_out
      geom2.vertadr = offset2
      geom2.vertnum = dim + 1
      geom2.graphadr = -1
      geom2.index = -1

      center1 = wp.vec3(0.0)
      for idx in range(dim + 1):
        center1 += workspace_verts_out[offset1 + idx]
      center1 = center1 / float(dim + 1)

      center2 = wp.vec3(0.0)
      for idx in range(dim + 1):
        center2 += workspace_verts_out[offset2 + idx]
      center2 = center2 / float(dim + 1)

      tol = opt_ccd_tolerance[worldid % opt_ccd_tolerance.shape[0]]

      dist, ncontact, w1, w2, _ = ccd(
        tol,
        2.0 * radius,
        gjk_iterations,
        epa_iterations,
        geom1,
        geom2,
        int(GeomType.MESH),
        int(GeomType.MESH),
        center1,
        center2,
        epa_vert_out[unique_thread_id],
        epa_vert_index_out[unique_thread_id],
        epa_face_out[unique_thread_id],
        epa_pr_out[unique_thread_id],
        epa_norm2_out[unique_thread_id],
        epa_horizon_out[unique_thread_id],
      )

      phys_dist = dist
      if ncontact > 0 and phys_dist < 0.0:
        p1_0 = workspace_verts_out[offset1]
        p1_1 = workspace_verts_out[offset1 + 1]
        p1_2 = workspace_verts_out[offset1 + 2]
        p2_0 = workspace_verts_out[offset2]
        p2_1 = workspace_verts_out[offset2 + 1]
        p2_2 = workspace_verts_out[offset2 + 2]
        if not (_inside_triangle(w1, p1_0, p1_1, p1_2, 0.2) and _inside_triangle(w2, p2_0, p2_1, p2_2, 0.2)):
          continue

        pos = 0.5 * (w1 + w2)
        nrm = wp.normalize(w1 - w2)
        _write_candidate_contact(
          max_candidates,
          phys_dist,
          pos,
          nrm,
          -2,
          flexid,
          e1,
          e2,
          worldid,
          overflow_out,
          cand_dist_out,
          cand_pos_out,
          cand_nrm_out,
          cand_geom_out,
          cand_flex_out,
          cand_elem_out,
          cand_vert_out,
          cand_worldid_out,
          cand_type_out,
          cand_geomcollisionid_out,
          ncand_out,
        )


@wp.kernel
def _flex_narrowphase_unified(
  # Model:
  ngeom: int,
  nflex: int,
  opt_ccd_tolerance: wp.array[float],
  opt_warn_overflow: bool,
  geom_type: wp.array[int],
  geom_condim: wp.array[int],
  geom_dataid: wp.array2d[int],
  geom_priority: wp.array[int],
  geom_solmix: wp.array2d[float],
  geom_solref: wp.array2d[wp.vec2],
  geom_solimp: wp.array2d[vec5],
  geom_size: wp.array2d[wp.vec3],
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
  flex_dim: wp.array[int],
  flex_vertadr: wp.array[int],
  flex_radius: wp.array[float],
  mesh_vertadr: wp.array[int],
  mesh_vertnum: wp.array[int],
  mesh_graphadr: wp.array[int],
  mesh_vert: wp.array[wp.vec3],
  mesh_graph: wp.array[int],
  mesh_pos: wp.array[wp.vec3],
  mesh_polynormal: wp.array[wp.vec3],
  mesh_polyvertadr: wp.array[int],
  mesh_polyvert: wp.array[int],
  mesh_polymapadr: wp.array[int],
  mesh_polymapnum: wp.array[int],
  mesh_polymap: wp.array[int],
  # Data in:
  geom_xpos_in: wp.array2d[wp.vec3],
  geom_xmat_in: wp.array2d[wp.mat33],
  flexvert_xpos_in: wp.array2d[wp.vec3],
  nworld_in: int,
  naconmax_in: int,
  naccdmax_in: int,
  ncollision_in: wp.array[int],
  # In:
  triadr: wp.array[int],
  flex_tridataadr: wp.array[int],
  flex_tri: wp.array[int],
  flex_triflexid: wp.array[int],
  collision_pair_in: wp.array[wp.vec2i],
  collision_worldid_in: wp.array[int],
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
  cand_type_out: wp.array[int],
  cand_geomcollisionid_out: wp.array[int],
  ncand_out: wp.array[int],
):
  collisionid = wp.tid()
  if collisionid >= ncollision_in[0] or collisionid >= collision_pair_in.shape[0]:
    return

  pair = collision_pair_in[collisionid]
  tri_id = pair[0]
  geomid = pair[1]

  gtype = geom_type[geomid]
  if (
    gtype != int(GeomType.SPHERE)
    and gtype != int(GeomType.CAPSULE)
    and gtype != int(GeomType.BOX)
    and gtype != int(GeomType.CYLINDER)
    and gtype != int(GeomType.MESH)
    and gtype != int(GeomType.ELLIPSOID)
  ):
    return

  worldid = collision_worldid_in[collisionid]

  flexid = flex_triflexid[tri_id]

  vert_adr = flex_vertadr[flexid]
  tri_radius = flex_radius[flexid]
  tri_margin = flex_margin[flexid]

  local_tri_id = tri_id - triadr[flexid]
  tri_data_idx = flex_tridataadr[flexid] + local_tri_id * 3
  v0_local = flex_tri[tri_data_idx]
  v1_local = flex_tri[tri_data_idx + 1]
  v2_local = flex_tri[tri_data_idx + 2]

  t1 = flexvert_xpos_in[worldid, vert_adr + v0_local]
  t2 = flexvert_xpos_in[worldid, vert_adr + v1_local]
  t3 = flexvert_xpos_in[worldid, vert_adr + v2_local]

  geom_margin_val = geom_margin[worldid % geom_margin.shape[0], geomid]
  margin = geom_margin_val + tri_margin

  geom_pos = geom_xpos_in[worldid, geomid]
  geom_rot = geom_xmat_in[worldid, geomid]
  geom_size_val = geom_size[worldid % geom_size.shape[0], geomid]

  if gtype == int(GeomType.MESH):
    ccdid = wp.atomic_add(nccd, 0, 1)
    if ccdid >= naccdmax_in:
      if opt_warn_overflow:
        wp.printf("CCD overflow in flex narrowphase - please increase naccdmax to %u\n", ccdid)
      wp.atomic_or(overflow_out, worldid, wp.static(OverflowType.CCD))
    else:
      did = geom_dataid[worldid % geom_dataid.shape[0], geomid]
      tolerance = opt_ccd_tolerance[worldid % opt_ccd_tolerance.shape[0]]
      _collide_mesh_triangle(
        mesh_vertadr,
        mesh_vertnum,
        mesh_graphadr,
        mesh_vert,
        mesh_graph,
        mesh_pos,
        mesh_polynormal,
        mesh_polyvertadr,
        mesh_polyvert,
        mesh_polymapadr,
        mesh_polymapnum,
        mesh_polymap,
        max_candidates,
        geom_pos,
        geom_rot,
        geom_size_val,
        t1,
        t2,
        t3,
        tri_radius,
        margin,
        geomid,
        flexid,
        local_tri_id,
        v0_local,
        v1_local,
        v2_local,
        worldid,
        did,
        epa_vert[ccdid],
        epa_vert_index[ccdid],
        epa_face[ccdid],
        epa_pr[ccdid],
        epa_norm2[ccdid],
        epa_horizon[ccdid],
        tolerance,
        ccd_iterations,
        overflow_out,
        cand_dist_out,
        cand_pos_out,
        cand_nrm_out,
        cand_geom_out,
        cand_flex_out,
        cand_elem_out,
        cand_vert_out,
        cand_worldid_out,
        cand_type_out,
        cand_geomcollisionid_out,
        ncand_out,
      )
  elif gtype == int(GeomType.ELLIPSOID):
    ccdid = wp.atomic_add(nccd, 0, 1)
    if ccdid >= naccdmax_in:
      if opt_warn_overflow:
        wp.printf("CCD overflow in flex narrowphase - please increase naccdmax to %u\n", ccdid)
      wp.atomic_or(overflow_out, worldid, wp.static(OverflowType.CCD))
    else:
      tolerance = opt_ccd_tolerance[worldid % opt_ccd_tolerance.shape[0]]

      # Construct Ellipsoid Geom (geom1)
      geom1 = Geom()
      geom1.pos = geom_pos
      geom1.rot = geom_rot
      geom1.size = geom_size_val
      geom1.margin = 0.0
      geom1.index = -1

      # Construct Triangle Geom (geom2)
      geom2 = Geom()
      geom2.pos = wp.vec3(0.0, 0.0, 0.0)
      geom2.rot = wp.mat33(t1[0], t1[1], t1[2], t2[0], t2[1], t2[2], t3[0], t3[1], t3[2])
      geom2.margin = 0.0
      geom2.index = -1

      centroid = (t1 + t2 + t3) * wp.static(1.0 / 3.0)
      r_geom = wp.length(geom_size_val)
      d1 = wp.length(t1 - centroid)
      d2 = wp.length(t2 - centroid)
      d3 = wp.length(t3 - centroid)
      r_tri = wp.max(d1, wp.max(d2, d3))

      if wp.length(centroid - geom_pos) <= r_geom + r_tri + margin + tri_radius + 0.04:
        dist, ncontact, w1, w2, idx = ccd(
          tolerance,
          margin + tri_radius,
          ccd_iterations,
          ccd_iterations,
          geom1,
          geom2,
          int(GeomType.ELLIPSOID),
          int(GeomType.TRIANGLE),
          geom_pos,
          centroid,
          epa_vert[ccdid],
          epa_vert_index[ccdid],
          epa_face[ccdid],
          epa_pr[ccdid],
          epa_norm2[ccdid],
          epa_horizon[ccdid],
        )

        if ncontact > 0 and dist < margin + tri_radius:
          if _inside_triangle(w2, t1, t2, t3, 0.2):
            if dist < 0.0:
              normal = wp.normalize(w1 - w2)
            else:
              normal = wp.normalize(w2 - w1)

            # Project triangle vertices onto normal to find deepest penetration
            dist_v0 = wp.dot(t1 - w1, normal) - tri_radius
            dist_v1 = wp.dot(t2 - w1, normal) - tri_radius
            dist_v2 = wp.dot(t3 - w1, normal) - tri_radius

            min_dist = wp.min(dist_v0, wp.min(dist_v1, dist_v2))
            if min_dist < margin:
              deepest_vert = v0_local
              pos = t1 - normal * (tri_radius + 0.5 * dist_v0)
              if dist_v1 < dist_v0 and dist_v1 < dist_v2:
                deepest_vert = v1_local
                pos = t2 - normal * (tri_radius + 0.5 * dist_v1)
              elif dist_v2 < dist_v0 and dist_v2 < dist_v1:
                deepest_vert = v2_local
                pos = t3 - normal * (tri_radius + 0.5 * dist_v2)

              _write_candidate_contact(
                max_candidates,
                min_dist,
                pos,
                normal,
                geomid,
                flexid,
                local_tri_id,
                -1,
                worldid,
                overflow_out,
                cand_dist_out,
                cand_pos_out,
                cand_nrm_out,
                cand_geom_out,
                cand_flex_out,
                cand_elem_out,
                cand_vert_out,
                cand_worldid_out,
                cand_type_out,
                cand_geomcollisionid_out,
                ncand_out,
              )
  else:
    _collide_geom_triangle_detect(
      max_candidates,
      gtype,
      geom_pos,
      geom_rot,
      geom_size_val,
      t1,
      t2,
      t3,
      tri_radius,
      margin,
      geomid,
      flexid,
      local_tri_id,
      -1,
      worldid,
      overflow_out,
      cand_dist_out,
      cand_pos_out,
      cand_nrm_out,
      cand_geom_out,
      cand_flex_out,
      cand_elem_out,
      cand_vert_out,
      cand_worldid_out,
      cand_type_out,
      cand_geomcollisionid_out,
      ncand_out,
    )


@wp.kernel
def _flex_narrowphase_tet_detect(
  # Model:
  ngeom: int,
  nflex: int,
  geom_type: wp.array[int],
  geom_contype: wp.array[int],
  geom_conaffinity: wp.array[int],
  geom_size: wp.array2d[wp.vec3],
  geom_margin: wp.array2d[float],
  flex_contype: wp.array[int],
  flex_conaffinity: wp.array[int],
  flex_margin: wp.array[float],
  flex_dim: wp.array[int],
  flex_vertadr: wp.array[int],
  flex_elemadr: wp.array[int],
  flex_elemnum: wp.array[int],
  flex_elemdataadr: wp.array[int],
  flex_elem: wp.array[int],
  flex_radius: wp.array[float],
  # Data in:
  geom_xpos_in: wp.array2d[wp.vec3],
  geom_xmat_in: wp.array2d[wp.mat33],
  flexvert_xpos_in: wp.array2d[wp.vec3],
  nworld_in: int,
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
  cand_type_out: wp.array[int],
  cand_geomcollisionid_out: wp.array[int],
  ncand_out: wp.array[int],
):
  worldid, elemid = wp.tid()

  # Find which flex owns this element
  flexid = int(-1)
  for i in range(nflex):
    if flex_dim[i] != 3:
      continue
    elem_adr = flex_elemadr[i]
    elem_num = flex_elemnum[i]
    if elemid >= elem_adr and elemid < elem_adr + elem_num:
      flexid = i
      break

  if flexid < 0:
    return

  vert_adr = flex_vertadr[flexid]
  tri_radius = flex_radius[flexid]
  tri_margin = flex_margin[flexid]

  # Extract 4 tet vertex indices (dim+1 = 4 for dim=3)
  local_elemid = elemid - flex_elemadr[flexid]
  edata_idx = flex_elemdataadr[flexid] + local_elemid * 4
  v0 = flex_elem[edata_idx]
  v1 = flex_elem[edata_idx + 1]
  v2 = flex_elem[edata_idx + 2]
  v3 = flex_elem[edata_idx + 3]

  # Fetch world-space vertex positions
  p0 = flexvert_xpos_in[worldid, vert_adr + v0]
  p1 = flexvert_xpos_in[worldid, vert_adr + v1]
  p2 = flexvert_xpos_in[worldid, vert_adr + v2]
  p3 = flexvert_xpos_in[worldid, vert_adr + v3]

  # TODO: Add a broadphase
  for geomid in range(ngeom):
    gtype = geom_type[geomid]
    if (
      gtype != int(GeomType.SPHERE)
      and gtype != int(GeomType.CAPSULE)
      and gtype != int(GeomType.BOX)
      and gtype != int(GeomType.CYLINDER)
    ):
      continue

    g_contype = geom_contype[geomid]
    g_conaffinity = geom_conaffinity[geomid]
    f_contype = flex_contype[flexid]
    f_conaffinity = flex_conaffinity[flexid]
    if not ((g_contype & f_conaffinity) or (f_contype & g_conaffinity)):
      continue

    geom_margin_val = geom_margin[worldid % geom_margin.shape[0], geomid]
    margin = geom_margin_val + tri_margin

    geom_pos = geom_xpos_in[worldid, geomid]
    geom_rot = geom_xmat_in[worldid, geomid]
    geom_size_val = geom_size[worldid % geom_size.shape[0], geomid]

    # Test all 4 triangular faces of the tet against the geom.
    # Face k is the triangle opposite vertex k:
    #   Face 0: (v1, v2, v3)
    #   Face 1: (v0, v2, v3)
    #   Face 2: (v0, v1, v3)
    #   Face 3: (v0, v1, v2)
    for face in range(4):
      if face == 0:
        t1 = p1
        t2 = p2
        t3 = p3
      elif face == 1:
        t1 = p0
        t2 = p2
        t3 = p3
      elif face == 2:
        t1 = p0
        t2 = p1
        t3 = p3
      else:
        t1 = p0
        t2 = p1
        t3 = p2

      _collide_geom_triangle_detect(
        max_candidates,
        gtype,
        geom_pos,
        geom_rot,
        geom_size_val,
        t1,
        t2,
        t3,
        tri_radius,
        margin,
        geomid,
        flexid,
        elemid,
        -1,
        worldid,
        overflow_out,
        cand_dist_out,
        cand_pos_out,
        cand_nrm_out,
        cand_geom_out,
        cand_flex_out,
        cand_elem_out,
        cand_vert_out,
        cand_worldid_out,
        cand_type_out,
        cand_geomcollisionid_out,
        ncand_out,
      )


@wp.kernel
def _compute_filter_key(
  # Model:
  ngeom: int,
  nflex: int,
  # In:
  ncand: wp.array[int],
  cand_geom: wp.array[wp.vec2i],
  cand_flex: wp.array[wp.vec2i],
  cand_worldid: wp.array[int],
  # Out:
  key_out: wp.array[int],
  val_out: wp.array[int],
):
  """Compute sort key for candidate grouping.

  Groups candidates by (worldid, flex_id, geom_id) so that duplicates
  are contiguous after sorting. Self-collision contacts (geom_id < 0)
  are mapped to a sentinel value (ngeom).
  """
  i = wp.tid()
  if i >= ncand[0]:
    key_out[i] = 2147483647  # INT_MAX: sort unused entries to end
    val_out[i] = i
    return

  worldid = cand_worldid[i]
  flex_id = cand_flex[i][1]
  geom_id = cand_geom[i][0]

  # Map self-collision (geom_id < 0) to sentinel
  g = geom_id
  if g < 0:
    g = ngeom

  key_out[i] = worldid * (nflex + 1) * (ngeom + 2) + flex_id * (ngeom + 2) + g
  val_out[i] = i


@wp.kernel
def _filter_flex_candidates_sorted(
  # In:
  ncand: wp.array[int],
  epsilon: float,
  sort_key: wp.array[int],
  sort_val: wp.array[int],
  cand_dist: wp.array[float],
  cand_pos: wp.array[wp.vec3],
  # Out:
  cand_active_out: wp.array[int],
):
  """Filter duplicate candidates using sorted order.

  After sorting by group key, candidates in the same group are contiguous.
  Each candidate only compares with neighbors sharing the same key, reducing
  complexity from O(n^2) to O(n * k) where k is the average group size.
  """
  si = wp.tid()
  if si >= ncand[0]:
    return

  i = sort_val[si]
  my_key = sort_key[si]
  pos_i = cand_pos[i]
  dist_i = cand_dist[i]
  eps2 = epsilon * epsilon

  keep = int(1)

  # Compare with same-key neighbors (backward)
  j = si - 1
  while j >= 0:
    if sort_key[j] != my_key:
      break
    oj = sort_val[j]
    diff = pos_i - cand_pos[oj]
    if wp.dot(diff, diff) < eps2:
      dist_j = cand_dist[oj]
      if dist_j < dist_i:
        keep = 0
      elif dist_j == dist_i and oj < i:
        keep = 0
    j -= 1

  # Compare with same-key neighbors (forward)
  j = si + 1
  while j < ncand[0]:
    if sort_key[j] != my_key:
      break
    oj = sort_val[j]
    diff = pos_i - cand_pos[oj]
    if wp.dot(diff, diff) < eps2:
      dist_j = cand_dist[oj]
      if dist_j < dist_i:
        keep = 0
      elif dist_j == dist_i and oj < i:
        keep = 0
    j += 1

  cand_active_out[i] = keep


@wp.kernel
def _filter_flex_candidates(
  # In:
  max_candidates: int,
  ncand: wp.array[int],
  epsilon: float,
  cand_dist: wp.array[float],
  cand_pos: wp.array[wp.vec3],
  cand_geom: wp.array[wp.vec2i],
  cand_flex: wp.array[wp.vec2i],
  cand_worldid: wp.array[int],
  # Out:
  cand_active_out: wp.array[int],
):
  i = wp.tid()
  limit = ncand[0]
  if i >= limit:
    return

  geom_i = cand_geom[i][0]
  flex_i = cand_flex[i][1]
  world_i = cand_worldid[i]
  pos_i = cand_pos[i]
  dist_i = cand_dist[i]

  keep = int(1)
  for j in range(max_candidates):
    if j >= limit:
      break
    if j == i:
      continue
    geom_j = cand_geom[j][0]
    if (geom_i >= 0 and geom_j >= 0 and geom_j == geom_i) or (geom_i < 0 and geom_j < 0):
      flex_j = cand_flex[j][1]
      if flex_j == flex_i:
        world_j = cand_worldid[j]
        if world_j == world_i:
          pos_j = cand_pos[j]
          dist_j = cand_dist[j]

          diff = pos_i - pos_j
          if wp.dot(diff, diff) < epsilon * epsilon:
            if dist_j < dist_i:
              keep = 0
            elif dist_j == dist_i and j < i:
              keep = 0

  cand_active_out[i] = keep


@wp.kernel
def _write_filtered_contacts(
  # Model:
  opt_warn_overflow: bool,
  geom_type: wp.array[int],
  geom_condim: wp.array[int],
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
  flex_dim: wp.array[int],
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
  cand_type: wp.array[int],
  cand_geomcollisionid: wp.array[int],
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
  contact_worldid_out: wp.array[int],
  contact_type_out: wp.array[int],
  contact_geomcollisionid_out: wp.array[int],
  nacon_out: wp.array[int],
  # Data out:
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
    margin = 0.0
    gap = 0.0

    mixed_condim, _, solref, solimp, friction = _mix_flex_contact_params(
      flex_condim[flex1],
      flex_priority[flex1],
      flex_solmix[flex1],
      flex_solref[flex1],
      flex_solimp[flex1],
      flex_friction[flex1],
      0.0,
      flex_condim[flex2],
      flex_priority[flex2],
      flex_solmix[flex2],
      flex_solref[flex2],
      flex_solimp[flex2],
      flex_friction[flex2],
      0.0,
    )

    if cand_vert[i][0] >= 0 and flex_dim[flex1] == 3:
      condim = 1
    else:
      condim = mixed_condim

  if cand_dist[i] >= margin:
    return

  id_ = wp.atomic_add(nacon_out, 0, 1)
  if id_ >= naconmax_in:
    if opt_warn_overflow:
      wp.printf(
        "flex contact overflow - please increase naconmax to %u\n",
        id_ + 1,
      )
    wp.atomic_or(overflow_out, worldid, wp.static(OverflowType.NARROWPHASE))
    return

  contact_dist_out[id_] = cand_dist[i]
  contact_pos_out[id_] = cand_pos[i]
  contact_frame_out[id_] = make_frame(cand_nrm[i])
  if geomid >= 0 and geom_type[geomid] == int(GeomType.PLANE):
    contact_includemargin_out[id_] = margin - gap
  else:
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
  contact_type_out[id_] = cand_type[i]
  contact_geomcollisionid_out[id_] = cand_geomcollisionid[i]


def flex_broadphase(m: Model, d: Data):
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
      d.flexvert_xpos,
    ],
    outputs=[
      d.flex_aabb_min,
      d.flex_aabb_max,
    ],
  )


@event_scope
def flex_collision(m: Model, d: Data, ctx):
  """Runs collision detection between geoms and flex elements."""
  if m.nflex == 0:
    return

  # Deduplicated candidate buffers (allocated on GPU)
  cand_dist = wp.empty(d.naconmax, dtype=float)
  cand_pos = wp.empty(d.naconmax, dtype=wp.vec3)
  cand_nrm = wp.empty(d.naconmax, dtype=wp.vec3)
  cand_geom = wp.empty(d.naconmax, dtype=wp.vec2i)
  cand_flex = wp.empty(d.naconmax, dtype=wp.vec2i)
  cand_elem = wp.empty(d.naconmax, dtype=wp.vec2i)
  cand_vert = wp.empty(d.naconmax, dtype=wp.vec2i)
  cand_worldid = wp.empty(d.naconmax, dtype=int)
  cand_type = wp.empty(d.naconmax, dtype=int)
  cand_geomcollisionid = wp.empty(d.naconmax, dtype=int)

  ncand = wp.zeros(1, dtype=int)

  # EPA workspaces if mesh or self collisions are possible
  epa_iterations = m.opt.ccd_iterations
  if m.nmesh > 0 or m.has_ellipsoid_geom:
    mesh_epa_vert = wp.empty(shape=(d.naccdmax, 10 + 2 * epa_iterations), dtype=wp.vec3)
    mesh_epa_vert_index = wp.empty(shape=(d.naccdmax, 10 + 2 * epa_iterations), dtype=int)
    mesh_epa_face = wp.empty(shape=(d.naccdmax, 6 + MJ_MAX_EPAFACES * epa_iterations), dtype=int)
    mesh_epa_pr = wp.empty(shape=(d.naccdmax, 6 + MJ_MAX_EPAFACES * epa_iterations), dtype=wp.vec3)
    mesh_epa_norm2 = wp.empty(shape=(d.naccdmax, 6 + MJ_MAX_EPAFACES * epa_iterations), dtype=float)
    mesh_epa_horizon = wp.empty(shape=(d.naccdmax, MJ_MAX_EPAHORIZON), dtype=int)
    mesh_nccd = wp.zeros(1, dtype=int)
  else:
    mesh_epa_vert = wp.empty(shape=(1, 1), dtype=wp.vec3)
    mesh_epa_vert_index = wp.empty(shape=(1, 1), dtype=int)
    mesh_epa_face = wp.empty(shape=(1, 1), dtype=int)
    mesh_epa_pr = wp.empty(shape=(1, 1), dtype=wp.vec3)
    mesh_epa_norm2 = wp.empty(shape=(1, 1), dtype=float)
    mesh_epa_horizon = wp.empty(shape=(1, 1), dtype=int)
    mesh_nccd = wp.zeros(1, dtype=int)

  ncollision_dim2 = wp.zeros(1, dtype=int)
  ncollision_dim3 = wp.zeros(1, dtype=int)
  ncollision_plane = wp.zeros(1, dtype=int)

  if m.has_3d_flex:
    wp.launch(
      _flex_narrowphase_tet_detect,
      dim=(d.nworld, m.nflexelem),
      inputs=[
        m.ngeom,
        m.nflex,
        m.geom_type,
        m.geom_contype,
        m.geom_conaffinity,
        m.geom_size,
        m.geom_margin,
        m.flex_contype,
        m.flex_conaffinity,
        m.flex_margin,
        m.flex_dim,
        m.flex_vertadr,
        m.flex_elemadr,
        m.flex_elemnum,
        m.flex_elemdataadr,
        m.flex_elem,
        m.flex_radius,
        d.geom_xpos,
        d.geom_xmat,
        d.flexvert_xpos,
        d.nworld,
        d.naconmax,
      ],
      outputs=[
        d.overflow,
        cand_dist,
        cand_pos,
        cand_nrm,
        cand_geom,
        cand_flex,
        cand_elem,
        cand_vert,
        cand_worldid,
        cand_type,
        cand_geomcollisionid,
        ncand,
      ],
    )

  # Update dynamic flex object bounding boxes
  flex_broadphase(m, d)

  # 2D Flex Element Collisions
  if m.flexelem_geom_pair_filtered.shape[0] > 0:
    wp.launch(
      _flex_broadphase_unified,
      dim=(d.nworld, m.flexelem_geom_pair_filtered.shape[0]),
      inputs=[
        m.ngeom,
        m.nflex,
        m.opt.warn_overflow,
        m.geom_type,
        m.geom_size,
        m.geom_aabb,
        m.geom_rbound,
        m.geom_margin,
        m.flex_margin,
        m.flex_dim,
        m.flex_vertadr,
        m.flex_radius,
        d.geom_xpos,
        d.geom_xmat,
        d.flexvert_xpos,
        d.naconmax,
        d.flex_aabb_min,
        d.flex_aabb_max,
        m.flex_elemadr,
        m.flex_elemdataadr,
        m.flex_elem,
        m.flexelem_geom_pair_filtered,
        m.flex_elemflexid,
      ],
      outputs=[
        ncollision_dim2,
        d.overflow,
        ctx.collision_pair,
        ctx.collision_worldid,
      ],
    )

    wp.launch(
      _flex_narrowphase_unified,
      dim=d.naconmax,
      inputs=[
        m.ngeom,
        m.nflex,
        m.opt.ccd_tolerance,
        m.opt.warn_overflow,
        m.geom_type,
        m.geom_condim,
        m.geom_dataid,
        m.geom_priority,
        m.geom_solmix,
        m.geom_solref,
        m.geom_solimp,
        m.geom_size,
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
        m.flex_dim,
        m.flex_vertadr,
        m.flex_radius,
        m.mesh_vertadr,
        m.mesh_vertnum,
        m.mesh_graphadr,
        m.mesh_vert,
        m.mesh_graph,
        m.mesh_pos,
        m.mesh_polynormal,
        m.mesh_polyvertadr,
        m.mesh_polyvert,
        m.mesh_polymapadr,
        m.mesh_polymapnum,
        m.mesh_polymap,
        d.geom_xpos,
        d.geom_xmat,
        d.flexvert_xpos,
        d.nworld,
        d.naconmax,
        d.naccdmax,
        ncollision_dim2,
        m.flex_elemadr,
        m.flex_elemdataadr,
        m.flex_elem,
        m.flex_elemflexid,
        ctx.collision_pair,
        ctx.collision_worldid,
        mesh_epa_vert,
        mesh_epa_vert_index,
        mesh_epa_face,
        mesh_epa_pr,
        mesh_epa_norm2,
        mesh_epa_horizon,
        mesh_nccd,
        epa_iterations,
        d.naconmax,
      ],
      outputs=[
        d.overflow,
        cand_dist,
        cand_pos,
        cand_nrm,
        cand_geom,
        cand_flex,
        cand_elem,
        cand_vert,
        cand_worldid,
        cand_type,
        cand_geomcollisionid,
        ncand,
      ],
    )

  # Plane Vertex Collisions
  if m.flexvert_geom_pair_filtered.shape[0] > 0:
    wp.launch(
      _flex_broadphase_plane,
      dim=(d.nworld, m.flexvert_geom_pair_filtered.shape[0]),
      inputs=[
        m.ngeom,
        m.opt.warn_overflow,
        m.geom_type,
        m.geom_margin,
        m.flex_margin,
        m.flex_vertadr,
        m.flex_radius,
        m.flexvert_geom_pair_filtered,
        m.flex_vertflexid,
        d.geom_xpos,
        d.geom_xmat,
        d.flexvert_xpos,
        d.naconmax,
        d.flex_aabb_min,
        d.flex_aabb_max,
      ],
      outputs=[
        ncollision_plane,
        d.overflow,
        ctx.collision_pair,
        ctx.collision_worldid,
      ],
    )

    wp.launch(
      _flex_plane_narrowphase,
      dim=d.naconmax,
      inputs=[
        m.ngeom,
        m.nflexvert,
        m.geom_type,
        m.geom_condim,
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
        m.flex_vertadr,
        m.flex_radius,
        m.flex_vertflexid,
        d.geom_xpos,
        d.geom_xmat,
        d.flexvert_xpos,
        d.nworld,
        d.naconmax,
        ncollision_plane,
        ctx.collision_pair,
        ctx.collision_worldid,
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
        d.contact.worldid,
        d.contact.type,
        d.contact.geomcollisionid,
        d.nacon,
      ],
    )

  # Geom Vertex Collisions (Candidate-based deduplicated narrowphase)
  if m.nflexvert > 0:
    wp.launch(
      _flex_geom_vertex_narrowphase_detect,
      dim=(d.nworld, m.nflexvert),
      inputs=[
        m.ngeom,
        m.nflexvert,
        m.geom_type,
        m.geom_contype,
        m.geom_conaffinity,
        m.geom_size,
        m.geom_margin,
        m.flex_contype,
        m.flex_conaffinity,
        m.flex_margin,
        m.flex_dim,
        m.flex_vertadr,
        m.flex_radius,
        m.flex_vertflexid,
        d.geom_xpos,
        d.geom_xmat,
        d.flexvert_xpos,
        d.nworld,
        d.naconmax,
      ],
      outputs=[
        d.overflow,
        cand_dist,
        cand_pos,
        cand_nrm,
        cand_geom,
        cand_flex,
        cand_elem,
        cand_vert,
        cand_worldid,
        cand_type,
        cand_geomcollisionid,
        ncand,
      ],
    )

  if m.nflexevpair > 0:
    wp.launch(
      _flex_internal_collisions_detect,
      dim=(d.nworld, m.nflexevpair),
      inputs=[
        m.nflex,
        m.flex_margin,
        m.flex_internal,
        m.flex_dim,
        m.flex_vertadr,
        m.flex_elemdataadr,
        m.flex_evpairadr,
        m.flex_evpairnum,
        m.flex_elem,
        m.flex_evpair,
        m.flex_radius,
        m.flex_evpairflexid,
        d.flexvert_xpos,
        d.naconmax,
      ],
      outputs=[
        d.overflow,
        cand_dist,
        cand_pos,
        cand_nrm,
        cand_geom,
        cand_flex,
        cand_elem,
        cand_vert,
        cand_worldid,
        cand_type,
        cand_geomcollisionid,
        ncand,
      ],
    )

  if m.nflexelem > 0:
    wp.launch(
      _flex_tet_internal_collisions_detect,
      dim=(d.nworld, m.nflexelem),
      inputs=[
        m.nflex,
        m.flex_dim,
        m.flex_vertadr,
        m.flex_elemadr,
        m.flex_elemnum,
        m.flex_elemdataadr,
        m.flex_elem,
        m.flex_radius,
        m.flex_elemflexid,
        d.flexvert_xpos,
        d.naconmax,
      ],
      outputs=[
        d.overflow,
        cand_dist,
        cand_pos,
        cand_nrm,
        cand_geom,
        cand_flex,
        cand_elem,
        cand_vert,
        cand_worldid,
        cand_type,
        cand_geomcollisionid,
        ncand,
      ],
    )

  selfcollide_enabled = m.has_flex_selfcollide

  if selfcollide_enabled and m.nflexelem > 0:
    epa_iterations = m.opt.ccd_iterations

    # TODO(team): investigate optimal nflexelem threshold for SAP vs brute-force
    if m.nflexelem > 32:
      # --- SAP broadphase path ---
      nelem = m.nflexelem
      nworldelem = d.nworld * nelem

      # Fixed projection direction (same as sap_broadphase in collision_driver.py)
      # TODO(team): compute optimal SAP direction
      direction = wp.vec3(0.5935, 0.7790, 0.1235)
      direction = wp.normalize(direction)

      # Allocate SAP arrays
      sap_lower = wp.empty((d.nworld, nelem, 2), dtype=float)
      sap_upper = wp.empty((d.nworld, nelem), dtype=float)
      sap_sort_index = wp.empty((d.nworld, nelem, 2), dtype=int)
      sap_range_arr = wp.empty((d.nworld, nelem), dtype=int)
      sap_cumsum = wp.empty((d.nworld, nelem), dtype=int)
      sap_seg_index = wp.empty(d.nworld + 1, dtype=int)
      elem_aabb_lower = wp.empty((d.nworld, nelem), dtype=wp.vec3)
      elem_aabb_upper = wp.empty((d.nworld, nelem), dtype=wp.vec3)

      # Step 1: Project element AABBs onto direction
      wp.launch(
        _flex_sap_project,
        dim=(d.nworld, nelem),
        inputs=[
          m.nflex,
          m.flex_selfcollide,
          m.flex_dim,
          m.flex_vertadr,
          m.flex_elemadr,
          m.flex_elemdataadr,
          m.flex_elem,
          m.flex_radius,
          m.flex_elemflexid,
          d.flexvert_xpos,
          d.nworld,
          nelem,
          direction,
        ],
        outputs=[
          sap_lower.reshape((-1, nelem)),
          sap_upper,
          sap_sort_index.reshape((-1, nelem)),
          elem_aabb_lower,
          elem_aabb_upper,
          sap_seg_index,
        ],
      )

      # Step 2: Sort
      wp.utils.segmented_sort_pairs(
        sap_lower.reshape((-1, nelem)),
        sap_sort_index.reshape((-1, nelem)),
        nworldelem,
        sap_seg_index,
      )

      # Step 3: Range
      wp.launch(
        sap_range,
        dim=(d.nworld, nelem),
        inputs=[
          nelem,
          sap_lower.reshape((-1, nelem)),
          sap_upper,
          sap_sort_index.reshape((-1, nelem)),
        ],
        outputs=[
          sap_range_arr,
        ],
      )

      # Step 4: Prefix sum for load balancing
      wp.utils.array_scan(
        sap_range_arr.reshape(-1),
        sap_cumsum.reshape(-1),
        True,
      )

      # Step 5: SAP sweep - output pairs only (no narrowphase)
      nsweep = 5 * nworldelem

      npairs = wp.zeros(1, dtype=int)
      pair_elem1 = wp.empty(d.naconmax, dtype=int)
      pair_elem2 = wp.empty(d.naconmax, dtype=int)
      pair_worldid = wp.empty(d.naconmax, dtype=int)

      # Step 5a: Generic SAP sweep (shared with geom broadphase)
      raw_npairs = wp.zeros(1, dtype=int)
      raw_pair_elem1 = wp.empty(d.naconmax, dtype=int)
      raw_pair_elem2 = wp.empty(d.naconmax, dtype=int)
      raw_pair_worldid = wp.empty(d.naconmax, dtype=int)

      wp.launch(
        sap_sweep,
        dim=nsweep,
        inputs=[
          nelem,
          sap_sort_index.reshape((-1, nelem)),
          sap_cumsum.reshape(-1),
          nsweep,
          elem_aabb_lower,
          elem_aabb_upper,
          d.naconmax,
        ],
        outputs=[
          raw_npairs,
          raw_pair_elem1,
          raw_pair_elem2,
          raw_pair_worldid,
        ],
      )

      # Step 5b: Filter pairs (flex-specific: selfcollide, shared vertices)
      wp.launch(
        _flex_sap_filter,
        dim=d.naconmax,
        inputs=[
          m.flex_selfcollide,
          m.flex_dim,
          m.flex_vertadr,
          m.flex_elemadr,
          m.flex_elemnum,
          m.flex_elemdataadr,
          m.flex_vertbodyid,
          m.flex_elem,
          m.flex_elemflexid,
          raw_npairs,
          raw_pair_elem1,
          raw_pair_elem2,
          raw_pair_worldid,
        ],
        outputs=[
          npairs,
          pair_elem1,
          pair_elem2,
          pair_worldid,
        ],
      )

      # Step 6: Narrowphase on actual pairs only
      workspace_verts = wp.empty(d.naconmax * 8, dtype=wp.vec3)

      if m.max_flex_dim > 1:
        epa_vert = wp.empty(shape=(d.naconmax, 10 + 2 * epa_iterations), dtype=wp.vec3)
        epa_vert_index = wp.empty(shape=(d.naconmax, 10 + 2 * epa_iterations), dtype=int)
        epa_face = wp.empty(shape=(d.naconmax, 6 + MJ_MAX_EPAFACES * epa_iterations), dtype=int)
        epa_pr = wp.empty(shape=(d.naconmax, 6 + MJ_MAX_EPAFACES * epa_iterations), dtype=wp.vec3)
        epa_norm2 = wp.empty(shape=(d.naconmax, 6 + MJ_MAX_EPAFACES * epa_iterations), dtype=float)
        epa_horizon = wp.empty(shape=(d.naconmax, MJ_MAX_EPAHORIZON), dtype=int)
      else:
        epa_vert = wp.empty(shape=(1, 1), dtype=wp.vec3)
        epa_vert_index = wp.empty(shape=(1, 1), dtype=int)
        epa_face = wp.empty(shape=(1, 1), dtype=int)
        epa_pr = wp.empty(shape=(1, 1), dtype=wp.vec3)
        epa_norm2 = wp.empty(shape=(1, 1), dtype=float)
        epa_horizon = wp.empty(shape=(1, 1), dtype=int)

      wp.launch(
        _flex_selfcollision_narrowphase,
        dim=d.naconmax,
        inputs=[
          m.nflex,
          m.opt.ccd_tolerance,
          m.flex_dim,
          m.flex_vertadr,
          m.flex_elemadr,
          m.flex_elemdataadr,
          m.flex_elem,
          m.flex_radius,
          m.flex_elemflexid,
          d.flexvert_xpos,
          d.naconmax,
          m.opt.ccd_iterations,
          epa_iterations,
          npairs,
          pair_elem1,
          pair_elem2,
          pair_worldid,
          d.naconmax,
          m.nflexelem,
        ],
        outputs=[
          d.overflow,
          workspace_verts,
          epa_vert,
          epa_vert_index,
          epa_face,
          epa_pr,
          epa_norm2,
          epa_horizon,
          cand_dist,
          cand_pos,
          cand_nrm,
          cand_geom,
          cand_flex,
          cand_elem,
          cand_vert,
          cand_worldid,
          cand_type,
          cand_geomcollisionid,
          ncand,
        ],
      )

    else:
      # --- Brute-force fallback for small element counts ---
      workspace_verts = wp.empty(d.nworld * m.nflexelem * 8, dtype=wp.vec3)

      if m.max_flex_dim > 1:
        epa_vert = wp.empty(shape=(d.nworld * m.nflexelem, 10 + 2 * epa_iterations), dtype=wp.vec3)
        epa_vert_index = wp.empty(shape=(d.nworld * m.nflexelem, 10 + 2 * epa_iterations), dtype=int)
        epa_face = wp.empty(shape=(d.nworld * m.nflexelem, 6 + MJ_MAX_EPAFACES * epa_iterations), dtype=int)
        epa_pr = wp.empty(shape=(d.nworld * m.nflexelem, 6 + MJ_MAX_EPAFACES * epa_iterations), dtype=wp.vec3)
        epa_norm2 = wp.empty(shape=(d.nworld * m.nflexelem, 6 + MJ_MAX_EPAFACES * epa_iterations), dtype=float)
        epa_horizon = wp.empty(shape=(d.nworld * m.nflexelem, MJ_MAX_EPAHORIZON), dtype=int)
      else:
        epa_vert = wp.empty(shape=(1, 1), dtype=wp.vec3)
        epa_vert_index = wp.empty(shape=(1, 1), dtype=int)
        epa_face = wp.empty(shape=(1, 1), dtype=int)
        epa_pr = wp.empty(shape=(1, 1), dtype=wp.vec3)
        epa_norm2 = wp.empty(shape=(1, 1), dtype=float)
        epa_horizon = wp.empty(shape=(1, 1), dtype=int)

      wp.launch(
        _flex_active_element_collisions_detect,
        dim=(d.nworld, m.nflexelem),
        inputs=[
          m.nflex,
          m.opt.ccd_tolerance,
          m.flex_selfcollide,
          m.flex_dim,
          m.flex_vertadr,
          m.flex_elemadr,
          m.flex_elemnum,
          m.flex_elemdataadr,
          m.flex_vertbodyid,
          m.flex_elem,
          m.flex_radius,
          m.flex_elemflexid,
          d.flexvert_xpos,
          d.naconmax,
          m.opt.ccd_iterations,
          epa_iterations,
          m.nflexelem,
        ],
        outputs=[
          d.overflow,
          workspace_verts,
          epa_vert,
          epa_vert_index,
          epa_face,
          epa_pr,
          epa_norm2,
          epa_horizon,
          cand_dist,
          cand_pos,
          cand_nrm,
          cand_geom,
          cand_flex,
          cand_elem,
          cand_vert,
          cand_worldid,
          cand_type,
          cand_geomcollisionid,
          ncand,
        ],
      )

  # Filter duplicate contacts using sort-based deduplication.
  # Sort candidates by (worldid, flex_id, geom_id) so duplicates are contiguous,
  # then compare only within each group. This is O(n log n) vs the naive O(n^2).
  filter_key = wp.empty(d.naconmax * 2, dtype=int)
  filter_val = wp.empty(d.naconmax * 2, dtype=int)
  wp.launch(
    _compute_filter_key,
    dim=d.naconmax,
    inputs=[
      m.ngeom,
      m.nflex,
      ncand,
      cand_geom,
      cand_flex,
      cand_worldid,
    ],
    outputs=[
      filter_key,
      filter_val,
    ],
  )
  wp.utils.radix_sort_pairs(filter_key, filter_val, d.naconmax)

  cand_active = wp.empty(d.naconmax, dtype=int)
  wp.launch(
    _filter_flex_candidates_sorted,
    dim=d.naconmax,
    inputs=[
      ncand,
      1e-3,  # epsilon
      filter_key,
      filter_val,
      cand_dist,
      cand_pos,
    ],
    outputs=[
      cand_active,
    ],
  )

  # Copy filtered contacts to the main d.contact array, computing contact parameters on-the-fly
  wp.launch(
    _write_filtered_contacts,
    dim=d.naconmax,
    inputs=[
      m.opt.warn_overflow,
      m.geom_type,
      m.geom_condim,
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
      m.flex_dim,
      d.naconmax,
      ncand,
      cand_dist,
      cand_pos,
      cand_nrm,
      cand_geom,
      cand_flex,
      cand_elem,
      cand_vert,
      cand_worldid,
      cand_type,
      cand_geomcollisionid,
      cand_active,
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
      d.contact.worldid,
      d.contact.type,
      d.contact.geomcollisionid,
      d.nacon,
      d.overflow,
    ],
  )
