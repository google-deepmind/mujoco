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

from typing import Tuple

import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive_core import box_box
from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive_core import capsule_box
from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive_core import capsule_capsule
from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive_core import plane_box
from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive_core import plane_capsule
from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive_core import plane_cylinder
from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive_core import plane_ellipsoid
from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive_core import plane_sphere
from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive_core import sphere_box
from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive_core import sphere_capsule
from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive_core import sphere_cylinder
from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive_core import sphere_sphere
from mujoco.mjx.third_party.mujoco_warp._src.math import make_frame
from mujoco.mjx.third_party.mujoco_warp._src.math import safe_div
from mujoco.mjx.third_party.mujoco_warp._src.math import upper_trid_index
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MINMU
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MINVAL
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import GeomType
from mujoco.mjx.third_party.mujoco_warp._src.types import Model
from mujoco.mjx.third_party.mujoco_warp._src.types import vec5
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import cache_kernel
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import kernel as nested_kernel

wp.set_module_options({"enable_backward": False})

_HUGE_VAL = 1e6


class mat43f(wp.types.matrix(shape=(4, 3), dtype=wp.float32)):
  pass


mat63 = wp.types.matrix(shape=(6, 3), dtype=float)


@wp.struct
class Geom:
  pos: wp.vec3
  rot: wp.mat33
  normal: wp.vec3
  size: wp.vec3
  margin: float
  hfprism: mat63
  vertadr: int
  vertnum: int
  vert: wp.array(dtype=wp.vec3)
  graphadr: int
  graph: wp.array(dtype=int)
  mesh_polynum: int
  mesh_polyadr: int
  mesh_polynormal: wp.array(dtype=wp.vec3)
  mesh_polyvertadr: wp.array(dtype=int)
  mesh_polyvertnum: wp.array(dtype=int)
  mesh_polyvert: wp.array(dtype=int)
  mesh_polymapadr: wp.array(dtype=int)
  mesh_polymapnum: wp.array(dtype=int)
  mesh_polymap: wp.array(dtype=int)
  index: int


@wp.func
def geom(
  # kernel_analyzer: off
  # Model:
  geom_type: int,
  geom_dataid: int,
  geom_size: wp.vec3,
  mesh_vertadr: wp.array(dtype=int),
  mesh_vertnum: wp.array(dtype=int),
  mesh_vert: wp.array(dtype=wp.vec3),
  mesh_graphadr: wp.array(dtype=int),
  mesh_graph: wp.array(dtype=int),
  mesh_polynum: wp.array(dtype=int),
  mesh_polyadr: wp.array(dtype=int),
  mesh_polynormal: wp.array(dtype=wp.vec3),
  mesh_polyvertadr: wp.array(dtype=int),
  mesh_polyvertnum: wp.array(dtype=int),
  mesh_polyvert: wp.array(dtype=int),
  mesh_polymapadr: wp.array(dtype=int),
  mesh_polymapnum: wp.array(dtype=int),
  mesh_polymap: wp.array(dtype=int),
  # Data in:
  geom_xpos_in: wp.vec3,
  geom_xmat_in: wp.mat33,
  # kernel_analyzer: on
) -> Geom:
  geom = Geom()
  geom.pos = geom_xpos_in
  geom.rot = geom_xmat_in
  geom.size = geom_size
  geom.normal = wp.vec3(geom_xmat_in[0, 2], geom_xmat_in[1, 2], geom_xmat_in[2, 2])  # plane

  if geom_type == GeomType.MESH:
    if geom_dataid >= 0:
      geom.vertadr = mesh_vertadr[geom_dataid]
      geom.vertnum = mesh_vertnum[geom_dataid]
      geom.graphadr = mesh_graphadr[geom_dataid]
      geom.mesh_polynum = mesh_polynum[geom_dataid]
      geom.mesh_polyadr = mesh_polyadr[geom_dataid]
    else:
      geom.vertadr = -1
      geom.vertnum = -1
      geom.graphadr = -1
      geom.mesh_polynum = -1
      geom.mesh_polyadr = -1

    geom.vert = mesh_vert
    geom.graph = mesh_graph
    geom.mesh_polynormal = mesh_polynormal
    geom.mesh_polyvertadr = mesh_polyvertadr
    geom.mesh_polyvertnum = mesh_polyvertnum
    geom.mesh_polyvert = mesh_polyvert
    geom.mesh_polymapadr = mesh_polymapadr
    geom.mesh_polymapnum = mesh_polymapnum
    geom.mesh_polymap = mesh_polymap

  geom.index = -1
  geom.margin = 0.0

  return geom


@wp.func
def plane_convex(plane_normal: wp.vec3, plane_pos: wp.vec3, convex: Geom) -> Tuple[wp.vec4, mat43f, wp.vec3]:
  """Core contact geometry calculation for plane-convex collision.

  Args:
    plane_normal: Normal vector of the plane
    plane_pos: Position point on the plane
    convex: Convex geometry object containing position, rotation, and mesh data

  Returns:
    Tuple containing:
      contact_dist: Vector of contact distances (wp.inf for unpopulated contacts)
      contact_pos: Matrix of contact positions (one per row)
      contact_normal: Matrix of contact normal vectors (one per row)
  """

  contact_dist = wp.vec4(wp.inf)
  contact_pos = mat43f()
  contact_count = int(0)

  # get points in the convex frame
  plane_pos_local = wp.transpose(convex.rot) @ (plane_pos - convex.pos)
  n = wp.transpose(convex.rot) @ plane_normal

  # Store indices in vec4
  indices = wp.vec4i(-1, -1, -1, -1)

  # exhaustive search over all vertices
  if convex.graphadr == -1 or convex.vertnum < 10:
    # find first support point (a)
    max_support = wp.float32(-_HUGE_VAL)
    a = wp.vec3()
    for i in range(convex.vertnum):
      vert = convex.vert[convex.vertadr + i]
      support = wp.dot(plane_pos_local - vert, n)
      if support > max_support:
        max_support = support
        indices[0] = i
        a = vert

    if max_support < 0:
      return contact_dist, contact_pos, plane_normal

    threshold = max_support - 1e-3

    # find point (b) furthest from a
    b_dist = wp.float32(-_HUGE_VAL)
    b = wp.vec3()
    for i in range(convex.vertnum):
      vert = convex.vert[convex.vertadr + i]
      support = wp.dot(plane_pos_local - vert, n)
      dist_mask = wp.where(support > threshold, 0.0, -_HUGE_VAL)
      dist = wp.length_sq(a - vert) + dist_mask
      if dist > b_dist:
        indices[1] = i
        b_dist = dist
        b = vert

    # find point (c) furthest along axis orthogonal to a-b
    ab = wp.cross(n, a - b)
    c_dist = wp.float32(-_HUGE_VAL)
    c = wp.vec3()
    for i in range(convex.vertnum):
      vert = convex.vert[convex.vertadr + i]
      support = wp.dot(plane_pos_local - vert, n)
      dist_mask = wp.where(support > threshold, 0.0, -_HUGE_VAL)
      ap = a - vert
      dist = wp.abs(wp.dot(ap, ab)) + dist_mask
      if dist > c_dist:
        indices[2] = i
        c_dist = dist
        c = vert

    # find point (d) furthest from other triangle edges
    ac = wp.cross(n, a - c)
    bc = wp.cross(n, b - c)
    d_dist = wp.float32(-_HUGE_VAL)
    for i in range(convex.vertnum):
      vert = convex.vert[convex.vertadr + i]
      support = wp.dot(plane_pos_local - vert, n)
      dist_mask = wp.where(support > threshold, 0.0, -_HUGE_VAL)
      ap = a - vert
      bp = b - vert
      dist_ap = wp.abs(wp.dot(ap, ac)) + dist_mask
      dist_bp = wp.abs(wp.dot(bp, bc)) + dist_mask
      if dist_ap + dist_bp > d_dist:
        indices[3] = i
        d_dist = dist_ap + dist_bp

  else:
    numvert = convex.graph[convex.graphadr]
    vert_edgeadr = convex.graphadr + 2
    vert_globalid = convex.graphadr + 2 + numvert
    edge_localid = convex.graphadr + 2 + 2 * numvert

    # Find support points
    max_support = wp.float32(-_HUGE_VAL)

    # hillclimb until no change
    prev = int(-1)
    imax = int(0)

    while True:
      prev = int(imax)
      i = int(convex.graph[vert_edgeadr + imax])
      while convex.graph[edge_localid + i] >= 0:
        subidx = convex.graph[edge_localid + i]
        idx = convex.graph[vert_globalid + subidx]
        support = wp.dot(plane_pos_local - convex.vert[convex.vertadr + idx], n)
        if support > max_support:
          max_support = support
          imax = int(subidx)
        i += int(1)
      if imax == prev:
        break

    threshold = wp.max(0.0, max_support - 1e-3)

    a_dist = wp.float32(-_HUGE_VAL)
    while True:
      prev = int(imax)
      i = int(convex.graph[vert_edgeadr + imax])
      while convex.graph[edge_localid + i] >= 0:
        subidx = convex.graph[edge_localid + i]
        idx = convex.graph[vert_globalid + subidx]
        support = wp.dot(plane_pos_local - convex.vert[convex.vertadr + idx], n)
        dist = wp.where(support > threshold, support, -_HUGE_VAL)
        if dist > a_dist:
          a_dist = dist
          imax = int(subidx)
        i += int(1)
      if imax == prev:
        break
    imax_global = convex.graph[vert_globalid + imax]
    a = convex.vert[convex.vertadr + imax_global]
    indices[0] = imax_global

    # Find point b (furthest from a)
    b_dist = wp.float32(-_HUGE_VAL)
    while True:
      prev = int(imax)
      i = int(convex.graph[vert_edgeadr + imax])
      while convex.graph[edge_localid + i] >= 0:
        subidx = convex.graph[edge_localid + i]
        idx = convex.graph[vert_globalid + subidx]
        support = wp.dot(plane_pos_local - convex.vert[convex.vertadr + idx], n)
        dist_mask = wp.where(support > threshold, 0.0, -_HUGE_VAL)
        dist = wp.length_sq(a - convex.vert[convex.vertadr + idx]) + dist_mask
        if dist > b_dist:
          b_dist = dist
          imax = int(subidx)
        i += int(1)
      if imax == prev:
        break
    imax_global = convex.graph[vert_globalid + imax]
    b = convex.vert[convex.vertadr + imax_global]
    indices[1] = imax_global

    # Find point c (furthest along axis orthogonal to a-b)
    ab = wp.cross(n, a - b)
    c_dist = wp.float32(-_HUGE_VAL)
    while True:
      prev = int(imax)
      i = int(convex.graph[vert_edgeadr + imax])
      while convex.graph[edge_localid + i] >= 0:
        subidx = convex.graph[edge_localid + i]
        idx = convex.graph[vert_globalid + subidx]
        support = wp.dot(plane_pos_local - convex.vert[convex.vertadr + idx], n)
        dist_mask = wp.where(support > threshold, 0.0, -_HUGE_VAL)
        ap = a - convex.vert[convex.vertadr + idx]
        dist = wp.abs(wp.dot(ap, ab)) + dist_mask
        if dist > c_dist:
          c_dist = dist
          imax = int(subidx)
        i += int(1)
      if imax == prev:
        break
    imax_global = convex.graph[vert_globalid + imax]
    c = convex.vert[convex.vertadr + imax_global]
    indices[2] = imax_global

    # Find point d (furthest from other triangle edges)
    ac = wp.cross(n, a - c)
    bc = wp.cross(n, b - c)
    d_dist = wp.float32(-_HUGE_VAL)
    while True:
      prev = int(imax)
      i = int(convex.graph[vert_edgeadr + imax])
      while convex.graph[edge_localid + i] >= 0:
        subidx = convex.graph[edge_localid + i]
        idx = convex.graph[vert_globalid + subidx]
        support = wp.dot(plane_pos_local - convex.vert[convex.vertadr + idx], n)
        dist_mask = wp.where(support > threshold, 0.0, -_HUGE_VAL)
        ap = a - convex.vert[convex.vertadr + idx]
        bp = b - convex.vert[convex.vertadr + idx]
        dist_ap = wp.abs(wp.dot(ap, ac)) + dist_mask
        dist_bp = wp.abs(wp.dot(bp, bc)) + dist_mask
        if dist_ap + dist_bp > d_dist:
          d_dist = dist_ap + dist_bp
          imax = int(subidx)
        i += int(1)
      if imax == prev:
        break
    imax_global = convex.graph[vert_globalid + imax]
    indices[3] = imax_global

  # Collect contacts from unique indices
  for i in range(3, -1, -1):
    idx = indices[i]
    count = int(0)
    for j in range(i + 1):
      if indices[j] == idx:
        count = count + 1

    # Check if the index is unique (appears exactly once)
    if count == 1:
      pos = convex.vert[convex.vertadr + idx]
      pos = convex.pos + convex.rot @ pos
      support = wp.dot(plane_pos_local - convex.vert[convex.vertadr + idx], n)
      dist = -support
      pos = pos - 0.5 * dist * plane_normal

      contact_dist[contact_count] = dist
      contact_pos[contact_count] = pos
      contact_count = contact_count + 1

  return contact_dist, contact_pos, plane_normal


@wp.func
def write_contact(
  # Data in:
  naconmax_in: int,
  # In:
  dist_in: float,
  pos_in: wp.vec3,
  frame_in: wp.mat33,
  margin_in: float,
  gap_in: float,
  condim_in: int,
  friction_in: vec5,
  solref_in: wp.vec2,
  solreffriction_in: wp.vec2,
  solimp_in: vec5,
  geoms_in: wp.vec2i,
  worldid_in: int,
  # Data out:
  nacon_out: wp.array(dtype=int),
  contact_dist_out: wp.array(dtype=float),
  contact_pos_out: wp.array(dtype=wp.vec3),
  contact_frame_out: wp.array(dtype=wp.mat33),
  contact_includemargin_out: wp.array(dtype=float),
  contact_friction_out: wp.array(dtype=vec5),
  contact_solref_out: wp.array(dtype=wp.vec2),
  contact_solreffriction_out: wp.array(dtype=wp.vec2),
  contact_solimp_out: wp.array(dtype=vec5),
  contact_dim_out: wp.array(dtype=int),
  contact_geom_out: wp.array(dtype=wp.vec2i),
  contact_worldid_out: wp.array(dtype=int),
):
  if dist_in - margin_in < 0.0:
    cid = wp.atomic_add(nacon_out, 0, 1)
    if cid < naconmax_in:
      contact_dist_out[cid] = dist_in
      contact_pos_out[cid] = pos_in
      contact_frame_out[cid] = frame_in
      contact_geom_out[cid] = geoms_in
      contact_worldid_out[cid] = worldid_in
      includemargin = margin_in - gap_in
      contact_includemargin_out[cid] = includemargin
      contact_dim_out[cid] = condim_in
      contact_friction_out[cid] = friction_in
      contact_solref_out[cid] = solref_in
      contact_solreffriction_out[cid] = solreffriction_in
      contact_solimp_out[cid] = solimp_in


@wp.func
def contact_params(
  # Model:
  geom_condim: wp.array(dtype=int),
  geom_priority: wp.array(dtype=int),
  geom_solmix: wp.array2d(dtype=float),
  geom_solref: wp.array2d(dtype=wp.vec2),
  geom_solimp: wp.array2d(dtype=vec5),
  geom_friction: wp.array2d(dtype=wp.vec3),
  geom_margin: wp.array2d(dtype=float),
  geom_gap: wp.array2d(dtype=float),
  pair_dim: wp.array(dtype=int),
  pair_solref: wp.array2d(dtype=wp.vec2),
  pair_solreffriction: wp.array2d(dtype=wp.vec2),
  pair_solimp: wp.array2d(dtype=vec5),
  pair_margin: wp.array2d(dtype=float),
  pair_gap: wp.array2d(dtype=float),
  pair_friction: wp.array2d(dtype=vec5),
  # Data in:
  collision_pair_in: wp.array(dtype=wp.vec2i),
  collision_pairid_in: wp.array(dtype=int),
  # In:
  cid: int,
  worldid: int,
):
  geoms = collision_pair_in[cid]
  pairid = collision_pairid_in[cid]

  if pairid > -1:
    margin = pair_margin[worldid, pairid]
    gap = pair_gap[worldid, pairid]
    condim = pair_dim[pairid]
    friction = pair_friction[worldid, pairid]
    solref = pair_solref[worldid, pairid]
    solreffriction = pair_solreffriction[worldid, pairid]
    solimp = pair_solimp[worldid, pairid]
  else:
    g1 = geoms[0]
    g2 = geoms[1]

    solmix1 = geom_solmix[worldid, g1]
    solmix2 = geom_solmix[worldid, g2]

    condim1 = geom_condim[g1]
    condim2 = geom_condim[g2]

    # priority
    p1 = geom_priority[g1]
    p2 = geom_priority[g2]

    if p1 > p2:
      mix = 1.0
      condim = condim1
      max_geom_friction = geom_friction[worldid, g1]
    elif p2 > p1:
      mix = 0.0
      condim = condim2
      max_geom_friction = geom_friction[worldid, g2]
    else:
      mix = safe_div(solmix1, solmix1 + solmix2)
      mix = wp.where((solmix1 < MJ_MINVAL) and (solmix2 < MJ_MINVAL), 0.5, mix)
      mix = wp.where((solmix1 < MJ_MINVAL) and (solmix2 >= MJ_MINVAL), 0.0, mix)
      mix = wp.where((solmix1 >= MJ_MINVAL) and (solmix2 < MJ_MINVAL), 1.0, mix)
      condim = wp.max(condim1, condim2)
      max_geom_friction = wp.max(geom_friction[worldid, g1], geom_friction[worldid, g2])

    friction = vec5(
      wp.max(MJ_MINMU, max_geom_friction[0]),
      wp.max(MJ_MINMU, max_geom_friction[0]),
      wp.max(MJ_MINMU, max_geom_friction[1]),
      wp.max(MJ_MINMU, max_geom_friction[2]),
      wp.max(MJ_MINMU, max_geom_friction[2]),
    )

    if geom_solref[worldid, g1][0] > 0.0 and geom_solref[worldid, g2][0] > 0.0:
      solref = mix * geom_solref[worldid, g1] + (1.0 - mix) * geom_solref[worldid, g2]
    else:
      solref = wp.min(geom_solref[worldid, g1], geom_solref[worldid, g2])

    solreffriction = wp.vec2(0.0, 0.0)

    solimp = mix * geom_solimp[worldid, g1] + (1.0 - mix) * geom_solimp[worldid, g2]

    # geom priority is ignored
    margin = wp.max(geom_margin[worldid, g1], geom_margin[worldid, g2])
    gap = wp.max(geom_gap[worldid, g1], geom_gap[worldid, g2])

  return geoms, margin, gap, condim, friction, solref, solreffriction, solimp


@wp.func
def plane_sphere_wrapper(
  # Data in:
  naconmax_in: int,
  # In:
  plane: Geom,
  sphere: Geom,
  worldid: int,
  margin: float,
  gap: float,
  condim: int,
  friction: vec5,
  solref: wp.vec2,
  solreffriction: wp.vec2,
  solimp: vec5,
  geoms: wp.vec2i,
  # Data out:
  nacon_out: wp.array(dtype=int),
  contact_dist_out: wp.array(dtype=float),
  contact_pos_out: wp.array(dtype=wp.vec3),
  contact_frame_out: wp.array(dtype=wp.mat33),
  contact_includemargin_out: wp.array(dtype=float),
  contact_friction_out: wp.array(dtype=vec5),
  contact_solref_out: wp.array(dtype=wp.vec2),
  contact_solreffriction_out: wp.array(dtype=wp.vec2),
  contact_solimp_out: wp.array(dtype=vec5),
  contact_dim_out: wp.array(dtype=int),
  contact_geom_out: wp.array(dtype=wp.vec2i),
  contact_worldid_out: wp.array(dtype=int),
):
  """Calculates contact between a sphere and a plane."""
  dist, pos = plane_sphere(plane.normal, plane.pos, sphere.pos, sphere.size[0])

  if dist - margin < 0:
    write_contact(
      naconmax_in,
      dist,
      pos,
      make_frame(plane.normal),
      margin,
      gap,
      condim,
      friction,
      solref,
      solreffriction,
      solimp,
      geoms,
      worldid,
      nacon_out,
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
      contact_worldid_out,
    )


@wp.func
def sphere_sphere_wrapper(
  # Data in:
  naconmax_in: int,
  # In:
  sphere1: Geom,
  sphere2: Geom,
  worldid: int,
  margin: float,
  gap: float,
  condim: int,
  friction: vec5,
  solref: wp.vec2,
  solreffriction: wp.vec2,
  solimp: vec5,
  geoms: wp.vec2i,
  # Data out:
  nacon_out: wp.array(dtype=int),
  contact_dist_out: wp.array(dtype=float),
  contact_pos_out: wp.array(dtype=wp.vec3),
  contact_frame_out: wp.array(dtype=wp.mat33),
  contact_includemargin_out: wp.array(dtype=float),
  contact_friction_out: wp.array(dtype=vec5),
  contact_solref_out: wp.array(dtype=wp.vec2),
  contact_solreffriction_out: wp.array(dtype=wp.vec2),
  contact_solimp_out: wp.array(dtype=vec5),
  contact_dim_out: wp.array(dtype=int),
  contact_geom_out: wp.array(dtype=wp.vec2i),
  contact_worldid_out: wp.array(dtype=int),
):
  """Calculates contact between two spheres."""
  dist, pos, normal = sphere_sphere(sphere1.pos, sphere1.size[0], sphere2.pos, sphere2.size[0])

  if dist - margin < 0:
    write_contact(
      naconmax_in,
      dist,
      pos,
      make_frame(normal),
      margin,
      gap,
      condim,
      friction,
      solref,
      solreffriction,
      solimp,
      geoms,
      worldid,
      nacon_out,
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
      contact_worldid_out,
    )


@wp.func
def sphere_capsule_wrapper(
  # Data in:
  naconmax_in: int,
  # In:
  sphere: Geom,
  cap: Geom,
  worldid: int,
  margin: float,
  gap: float,
  condim: int,
  friction: vec5,
  solref: wp.vec2,
  solreffriction: wp.vec2,
  solimp: vec5,
  geoms: wp.vec2i,
  # Data out:
  nacon_out: wp.array(dtype=int),
  contact_dist_out: wp.array(dtype=float),
  contact_pos_out: wp.array(dtype=wp.vec3),
  contact_frame_out: wp.array(dtype=wp.mat33),
  contact_includemargin_out: wp.array(dtype=float),
  contact_friction_out: wp.array(dtype=vec5),
  contact_solref_out: wp.array(dtype=wp.vec2),
  contact_solreffriction_out: wp.array(dtype=wp.vec2),
  contact_solimp_out: wp.array(dtype=vec5),
  contact_dim_out: wp.array(dtype=int),
  contact_geom_out: wp.array(dtype=wp.vec2i),
  contact_worldid_out: wp.array(dtype=int),
):
  """Calculates one contact between a sphere and a capsule."""
  # capsule axis
  axis = wp.vec3(cap.rot[0, 2], cap.rot[1, 2], cap.rot[2, 2])

  dist, pos, normal = sphere_capsule(sphere.pos, sphere.size[0], cap.pos, axis, cap.size[0], cap.size[1])

  if dist - margin < 0:
    write_contact(
      naconmax_in,
      dist,
      pos,
      make_frame(normal),
      margin,
      gap,
      condim,
      friction,
      solref,
      solreffriction,
      solimp,
      geoms,
      worldid,
      nacon_out,
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
      contact_worldid_out,
    )


@wp.func
def capsule_capsule_wrapper(
  # Data in:
  naconmax_in: int,
  # In:
  cap1: Geom,
  cap2: Geom,
  worldid: int,
  margin: float,
  gap: float,
  condim: int,
  friction: vec5,
  solref: wp.vec2,
  solreffriction: wp.vec2,
  solimp: vec5,
  geoms: wp.vec2i,
  # Data out:
  nacon_out: wp.array(dtype=int),
  contact_dist_out: wp.array(dtype=float),
  contact_pos_out: wp.array(dtype=wp.vec3),
  contact_frame_out: wp.array(dtype=wp.mat33),
  contact_includemargin_out: wp.array(dtype=float),
  contact_friction_out: wp.array(dtype=vec5),
  contact_solref_out: wp.array(dtype=wp.vec2),
  contact_solreffriction_out: wp.array(dtype=wp.vec2),
  contact_solimp_out: wp.array(dtype=vec5),
  contact_dim_out: wp.array(dtype=int),
  contact_geom_out: wp.array(dtype=wp.vec2i),
  contact_worldid_out: wp.array(dtype=int),
):
  """Calculates contacts between two capsules."""
  # capsule axes
  cap1_axis = wp.vec3(cap1.rot[0, 2], cap1.rot[1, 2], cap1.rot[2, 2])
  cap2_axis = wp.vec3(cap2.rot[0, 2], cap2.rot[1, 2], cap2.rot[2, 2])

  dist, pos, normal = capsule_capsule(
    cap1.pos,
    cap1_axis,
    cap1.size[0],  # radius1
    cap1.size[1],  # half_length1
    cap2.pos,
    cap2_axis,
    cap2.size[0],  # radius2
    cap2.size[1],  # half_length2
  )

  if dist - margin < 0:
    write_contact(
      naconmax_in,
      dist,
      pos,
      make_frame(normal),
      margin,
      gap,
      condim,
      friction,
      solref,
      solreffriction,
      solimp,
      geoms,
      worldid,
      nacon_out,
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
      contact_worldid_out,
    )


@wp.func
def plane_capsule_wrapper(
  # Data in:
  naconmax_in: int,
  # In:
  plane: Geom,
  cap: Geom,
  worldid: int,
  margin: float,
  gap: float,
  condim: int,
  friction: vec5,
  solref: wp.vec2,
  solreffriction: wp.vec2,
  solimp: vec5,
  geoms: wp.vec2i,
  # Data out:
  nacon_out: wp.array(dtype=int),
  contact_dist_out: wp.array(dtype=float),
  contact_pos_out: wp.array(dtype=wp.vec3),
  contact_frame_out: wp.array(dtype=wp.mat33),
  contact_includemargin_out: wp.array(dtype=float),
  contact_friction_out: wp.array(dtype=vec5),
  contact_solref_out: wp.array(dtype=wp.vec2),
  contact_solreffriction_out: wp.array(dtype=wp.vec2),
  contact_solimp_out: wp.array(dtype=vec5),
  contact_dim_out: wp.array(dtype=int),
  contact_geom_out: wp.array(dtype=wp.vec2i),
  contact_worldid_out: wp.array(dtype=int),
):
  """Calculates contacts between a capsule and a plane."""
  # capsule axis
  capsule_axis = wp.vec3(cap.rot[0, 2], cap.rot[1, 2], cap.rot[2, 2])

  dist, pos, frame = plane_capsule(
    plane.normal,
    plane.pos,
    cap.pos,
    capsule_axis,
    cap.size[0],  # radius
    cap.size[1],  # half_length
  )

  for i in range(2):
    disti = dist[i]
    if disti - margin < 0.0:
      write_contact(
        naconmax_in,
        disti,
        pos[i],
        frame,
        margin,
        gap,
        condim,
        friction,
        solref,
        solreffriction,
        solimp,
        geoms,
        worldid,
        nacon_out,
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
        contact_worldid_out,
      )


@wp.func
def plane_ellipsoid_wrapper(
  # Data in:
  naconmax_in: int,
  # In:
  plane: Geom,
  ellipsoid: Geom,
  worldid: int,
  margin: float,
  gap: float,
  condim: int,
  friction: vec5,
  solref: wp.vec2,
  solreffriction: wp.vec2,
  solimp: vec5,
  geoms: wp.vec2i,
  # Data out:
  nacon_out: wp.array(dtype=int),
  contact_dist_out: wp.array(dtype=float),
  contact_pos_out: wp.array(dtype=wp.vec3),
  contact_frame_out: wp.array(dtype=wp.mat33),
  contact_includemargin_out: wp.array(dtype=float),
  contact_friction_out: wp.array(dtype=vec5),
  contact_solref_out: wp.array(dtype=wp.vec2),
  contact_solreffriction_out: wp.array(dtype=wp.vec2),
  contact_solimp_out: wp.array(dtype=vec5),
  contact_dim_out: wp.array(dtype=int),
  contact_geom_out: wp.array(dtype=wp.vec2i),
  contact_worldid_out: wp.array(dtype=int),
):
  """Calculates contacts between an ellipsoid and a plane."""
  dist, pos, normal = plane_ellipsoid(plane.normal, plane.pos, ellipsoid.pos, ellipsoid.rot, ellipsoid.size)

  if dist - margin < 0:
    write_contact(
      naconmax_in,
      dist,
      pos,
      make_frame(normal),
      margin,
      gap,
      condim,
      friction,
      solref,
      solreffriction,
      solimp,
      geoms,
      worldid,
      nacon_out,
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
      contact_worldid_out,
    )


@wp.func
def plane_box_wrapper(
  # Data in:
  naconmax_in: int,
  # In:
  plane: Geom,
  box: Geom,
  worldid: int,
  margin: float,
  gap: float,
  condim: int,
  friction: vec5,
  solref: wp.vec2,
  solreffriction: wp.vec2,
  solimp: vec5,
  geoms: wp.vec2i,
  # Data out:
  nacon_out: wp.array(dtype=int),
  contact_dist_out: wp.array(dtype=float),
  contact_pos_out: wp.array(dtype=wp.vec3),
  contact_frame_out: wp.array(dtype=wp.mat33),
  contact_includemargin_out: wp.array(dtype=float),
  contact_friction_out: wp.array(dtype=vec5),
  contact_solref_out: wp.array(dtype=wp.vec2),
  contact_solreffriction_out: wp.array(dtype=wp.vec2),
  contact_solimp_out: wp.array(dtype=vec5),
  contact_dim_out: wp.array(dtype=int),
  contact_geom_out: wp.array(dtype=wp.vec2i),
  contact_worldid_out: wp.array(dtype=int),
):
  """Calculates contacts between a box and a plane."""
  dist, pos, normal = plane_box(plane.normal, plane.pos, box.pos, box.rot, box.size)
  frame = make_frame(normal)

  for i in range(4):
    disti = dist[i]
    if disti - margin < 0.0:
      write_contact(
        naconmax_in,
        disti,
        pos[i],
        frame,
        margin,
        gap,
        condim,
        friction,
        solref,
        solreffriction,
        solimp,
        geoms,
        worldid,
        nacon_out,
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
        contact_worldid_out,
      )


_HUGE_VAL = 1e6


@wp.func
def plane_convex_wrapper(
  # Data in:
  naconmax_in: int,
  # In:
  plane: Geom,
  convex: Geom,
  worldid: int,
  margin: float,
  gap: float,
  condim: int,
  friction: vec5,
  solref: wp.vec2,
  solreffriction: wp.vec2,
  solimp: vec5,
  geoms: wp.vec2i,
  # Data out:
  nacon_out: wp.array(dtype=int),
  contact_dist_out: wp.array(dtype=float),
  contact_pos_out: wp.array(dtype=wp.vec3),
  contact_frame_out: wp.array(dtype=wp.mat33),
  contact_includemargin_out: wp.array(dtype=float),
  contact_friction_out: wp.array(dtype=vec5),
  contact_solref_out: wp.array(dtype=wp.vec2),
  contact_solreffriction_out: wp.array(dtype=wp.vec2),
  contact_solimp_out: wp.array(dtype=vec5),
  contact_dim_out: wp.array(dtype=int),
  contact_geom_out: wp.array(dtype=wp.vec2i),
  contact_worldid_out: wp.array(dtype=int),
):
  """Calculates contacts between a plane and a convex object."""
  dist, pos, normal = plane_convex(plane.normal, plane.pos, convex)

  frame = make_frame(normal)
  for i in range(4):
    disti = dist[i]
    if disti - margin < 0.0:
      write_contact(
        naconmax_in,
        disti,
        pos[i],
        frame,
        margin,
        gap,
        condim,
        friction,
        solref,
        solreffriction,
        solimp,
        geoms,
        worldid,
        nacon_out,
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
        contact_worldid_out,
      )


@wp.func
def sphere_cylinder_wrapper(
  # Data in:
  naconmax_in: int,
  # In:
  sphere: Geom,
  cylinder: Geom,
  worldid: int,
  margin: float,
  gap: float,
  condim: int,
  friction: vec5,
  solref: wp.vec2,
  solreffriction: wp.vec2,
  solimp: vec5,
  geoms: wp.vec2i,
  # Data out:
  nacon_out: wp.array(dtype=int),
  contact_dist_out: wp.array(dtype=float),
  contact_pos_out: wp.array(dtype=wp.vec3),
  contact_frame_out: wp.array(dtype=wp.mat33),
  contact_includemargin_out: wp.array(dtype=float),
  contact_friction_out: wp.array(dtype=vec5),
  contact_solref_out: wp.array(dtype=wp.vec2),
  contact_solreffriction_out: wp.array(dtype=wp.vec2),
  contact_solimp_out: wp.array(dtype=vec5),
  contact_dim_out: wp.array(dtype=int),
  contact_geom_out: wp.array(dtype=wp.vec2i),
  contact_worldid_out: wp.array(dtype=int),
):
  """Calculates contacts between a sphere and a cylinder."""
  # cylinder axis
  cylinder_axis = wp.vec3(cylinder.rot[0, 2], cylinder.rot[1, 2], cylinder.rot[2, 2])

  dist, pos, normal = sphere_cylinder(
    sphere.pos,
    sphere.size[0],  # sphere radius
    cylinder.pos,
    cylinder_axis,
    cylinder.size[0],  # cylinder radius
    cylinder.size[1],  # cylinder half_height
  )

  if dist - margin < 0.0:
    write_contact(
      naconmax_in,
      dist,
      pos,
      make_frame(normal),
      margin,
      gap,
      condim,
      friction,
      solref,
      solreffriction,
      solimp,
      geoms,
      worldid,
      nacon_out,
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
      contact_worldid_out,
    )


@wp.func
def plane_cylinder_wrapper(
  # Data in:
  naconmax_in: int,
  # In:
  plane: Geom,
  cylinder: Geom,
  worldid: int,
  margin: float,
  gap: float,
  condim: int,
  friction: vec5,
  solref: wp.vec2,
  solreffriction: wp.vec2,
  solimp: vec5,
  geoms: wp.vec2i,
  # Data out:
  nacon_out: wp.array(dtype=int),
  contact_dist_out: wp.array(dtype=float),
  contact_pos_out: wp.array(dtype=wp.vec3),
  contact_frame_out: wp.array(dtype=wp.mat33),
  contact_includemargin_out: wp.array(dtype=float),
  contact_friction_out: wp.array(dtype=vec5),
  contact_solref_out: wp.array(dtype=wp.vec2),
  contact_solreffriction_out: wp.array(dtype=wp.vec2),
  contact_solimp_out: wp.array(dtype=vec5),
  contact_dim_out: wp.array(dtype=int),
  contact_geom_out: wp.array(dtype=wp.vec2i),
  contact_worldid_out: wp.array(dtype=int),
):
  """Calculates contacts between a cylinder and a plane."""
  # cylinder axis
  cylinder_axis = wp.vec3(cylinder.rot[0, 2], cylinder.rot[1, 2], cylinder.rot[2, 2])

  dist, pos, normal = plane_cylinder(
    plane.normal,
    plane.pos,
    cylinder.pos,
    cylinder_axis,
    cylinder.size[0],  # radius
    cylinder.size[1],  # half_height
  )

  frame = make_frame(normal)
  for i in range(4):
    disti = dist[i]
    if disti - margin < 0.0:
      write_contact(
        naconmax_in,
        disti,
        pos[i],
        frame,
        margin,
        gap,
        condim,
        friction,
        solref,
        solreffriction,
        solimp,
        geoms,
        worldid,
        nacon_out,
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
        contact_worldid_out,
      )


@wp.func
def sphere_box_wrapper(
  # Data in:
  naconmax_in: int,
  # In:
  sphere: Geom,
  box: Geom,
  worldid: int,
  margin: float,
  gap: float,
  condim: int,
  friction: vec5,
  solref: wp.vec2,
  solreffriction: wp.vec2,
  solimp: vec5,
  geoms: wp.vec2i,
  # Data out:
  nacon_out: wp.array(dtype=int),
  contact_dist_out: wp.array(dtype=float),
  contact_pos_out: wp.array(dtype=wp.vec3),
  contact_frame_out: wp.array(dtype=wp.mat33),
  contact_includemargin_out: wp.array(dtype=float),
  contact_friction_out: wp.array(dtype=vec5),
  contact_solref_out: wp.array(dtype=wp.vec2),
  contact_solreffriction_out: wp.array(dtype=wp.vec2),
  contact_solimp_out: wp.array(dtype=vec5),
  contact_dim_out: wp.array(dtype=int),
  contact_geom_out: wp.array(dtype=wp.vec2i),
  contact_worldid_out: wp.array(dtype=int),
):
  dist, pos, normal = sphere_box(sphere.pos, sphere.size[0], box.pos, box.rot, box.size)

  if dist - margin < 0.0:
    write_contact(
      naconmax_in,
      dist,
      pos,
      make_frame(normal),
      margin,
      gap,
      condim,
      friction,
      solref,
      solreffriction,
      solimp,
      geoms,
      worldid,
      nacon_out,
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
      contact_worldid_out,
    )


@wp.func
def capsule_box_wrapper(
  # Data in:
  naconmax_in: int,
  # In:
  cap: Geom,
  box: Geom,
  worldid: int,
  margin: float,
  gap: float,
  condim: int,
  friction: vec5,
  solref: wp.vec2,
  solreffriction: wp.vec2,
  solimp: vec5,
  geoms: wp.vec2i,
  # Data out:
  nacon_out: wp.array(dtype=int),
  contact_dist_out: wp.array(dtype=float),
  contact_pos_out: wp.array(dtype=wp.vec3),
  contact_frame_out: wp.array(dtype=wp.mat33),
  contact_includemargin_out: wp.array(dtype=float),
  contact_friction_out: wp.array(dtype=vec5),
  contact_solref_out: wp.array(dtype=wp.vec2),
  contact_solreffriction_out: wp.array(dtype=wp.vec2),
  contact_solimp_out: wp.array(dtype=vec5),
  contact_dim_out: wp.array(dtype=int),
  contact_geom_out: wp.array(dtype=wp.vec2i),
  contact_worldid_out: wp.array(dtype=int),
):
  """Calculates contacts between a capsule and a box."""
  # Extract capsule axis
  axis = wp.vec3(cap.rot[0, 2], cap.rot[1, 2], cap.rot[2, 2])

  # Call the core function to get contact geometry
  dist, pos, normal = capsule_box(
    cap.pos,
    axis,
    cap.size[0],  # capsule radius
    cap.size[1],  # capsule half length
    box.pos,
    box.rot,
    box.size,
  )

  # Loop over the contacts and write them
  for i in range(2):
    disti = dist[i]
    if disti - margin < 0.0:
      write_contact(
        naconmax_in,
        disti,
        pos[i],
        make_frame(normal[i]),
        margin,
        gap,
        condim,
        friction,
        solref,
        solreffriction,
        solimp,
        geoms,
        worldid,
        nacon_out,
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
        contact_worldid_out,
      )


@wp.func
def box_box_wrapper(
  # Data in:
  naconmax_in: int,
  # In:
  box1: Geom,
  box2: Geom,
  worldid: int,
  margin: float,
  gap: float,
  condim: int,
  friction: vec5,
  solref: wp.vec2,
  solreffriction: wp.vec2,
  solimp: vec5,
  geoms: wp.vec2i,
  # Data out:
  nacon_out: wp.array(dtype=int),
  contact_dist_out: wp.array(dtype=float),
  contact_pos_out: wp.array(dtype=wp.vec3),
  contact_frame_out: wp.array(dtype=wp.mat33),
  contact_includemargin_out: wp.array(dtype=float),
  contact_friction_out: wp.array(dtype=vec5),
  contact_solref_out: wp.array(dtype=wp.vec2),
  contact_solreffriction_out: wp.array(dtype=wp.vec2),
  contact_solimp_out: wp.array(dtype=vec5),
  contact_dim_out: wp.array(dtype=int),
  contact_geom_out: wp.array(dtype=wp.vec2i),
  contact_worldid_out: wp.array(dtype=int),
):
  """Calculates contacts between two boxes."""
  # Call the core function to get contact geometry
  dist, pos, normal = box_box(
    box1.pos,
    box1.rot,
    box1.size,
    box2.pos,
    box2.rot,
    box2.size,
  )

  for i in range(8):
    if dist[i] - margin >= 0.0:
      continue

    write_contact(
      naconmax_in,
      dist[i],
      pos[i],
      make_frame(normal[i]),
      margin,
      gap,
      condim,
      friction,
      solref,
      solreffriction,
      solimp,
      geoms,
      worldid,
      nacon_out,
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
      contact_worldid_out,
    )


_PRIMITIVE_COLLISIONS = {
  (GeomType.PLANE, GeomType.SPHERE): plane_sphere_wrapper,
  (GeomType.PLANE, GeomType.CAPSULE): plane_capsule_wrapper,
  (GeomType.PLANE, GeomType.ELLIPSOID): plane_ellipsoid_wrapper,
  (GeomType.PLANE, GeomType.CYLINDER): plane_cylinder_wrapper,
  (GeomType.PLANE, GeomType.BOX): plane_box_wrapper,
  (GeomType.PLANE, GeomType.MESH): plane_convex_wrapper,
  (GeomType.SPHERE, GeomType.SPHERE): sphere_sphere_wrapper,
  (GeomType.SPHERE, GeomType.CAPSULE): sphere_capsule_wrapper,
  (GeomType.SPHERE, GeomType.CYLINDER): sphere_cylinder_wrapper,
  (GeomType.SPHERE, GeomType.BOX): sphere_box_wrapper,
  (GeomType.CAPSULE, GeomType.CAPSULE): capsule_capsule_wrapper,
  (GeomType.CAPSULE, GeomType.BOX): capsule_box_wrapper,
  (GeomType.BOX, GeomType.BOX): box_box_wrapper,
}


# TODO(team): _check_collisions shared utility
def _check_primitive_collisions():
  prev_idx = -1
  for types in _PRIMITIVE_COLLISIONS.keys():
    idx = upper_trid_index(len(GeomType), types[0].value, types[1].value)
    if types[1] < types[0] or idx <= prev_idx:
      return False
    prev_idx = idx
  return True


assert _check_primitive_collisions(), "_PRIMITIVE_COLLISIONS is in invalid order"


@cache_kernel
def _create_narrowphase_kernel(primitive_collisions_types, primitive_collisions_func):
  # AD: no unique here:
  # * we expect this generator to be called only once per model, so no repeated compilation
  # * module="unique" is generating problems because it uses the function name as the key
  #   that in turn will cause multiple kernels to be generated with the same name
  #   this is mostly problematic in cases like the UTs where we don't clear the kernel cache
  #   between different tests.

  @nested_kernel(enable_backward=False)
  def _primitive_narrowphase(
    # Model:
    geom_type: wp.array(dtype=int),
    geom_condim: wp.array(dtype=int),
    geom_dataid: wp.array(dtype=int),
    geom_priority: wp.array(dtype=int),
    geom_solmix: wp.array2d(dtype=float),
    geom_solref: wp.array2d(dtype=wp.vec2),
    geom_solimp: wp.array2d(dtype=vec5),
    geom_size: wp.array2d(dtype=wp.vec3),
    geom_friction: wp.array2d(dtype=wp.vec3),
    geom_margin: wp.array2d(dtype=float),
    geom_gap: wp.array2d(dtype=float),
    hfield_adr: wp.array(dtype=int),
    hfield_nrow: wp.array(dtype=int),
    hfield_ncol: wp.array(dtype=int),
    hfield_size: wp.array(dtype=wp.vec4),
    hfield_data: wp.array(dtype=float),
    mesh_vertadr: wp.array(dtype=int),
    mesh_vertnum: wp.array(dtype=int),
    mesh_vert: wp.array(dtype=wp.vec3),
    mesh_graphadr: wp.array(dtype=int),
    mesh_graph: wp.array(dtype=int),
    mesh_polynum: wp.array(dtype=int),
    mesh_polyadr: wp.array(dtype=int),
    mesh_polynormal: wp.array(dtype=wp.vec3),
    mesh_polyvertadr: wp.array(dtype=int),
    mesh_polyvertnum: wp.array(dtype=int),
    mesh_polyvert: wp.array(dtype=int),
    mesh_polymapadr: wp.array(dtype=int),
    mesh_polymapnum: wp.array(dtype=int),
    mesh_polymap: wp.array(dtype=int),
    pair_dim: wp.array(dtype=int),
    pair_solref: wp.array2d(dtype=wp.vec2),
    pair_solreffriction: wp.array2d(dtype=wp.vec2),
    pair_solimp: wp.array2d(dtype=vec5),
    pair_margin: wp.array2d(dtype=float),
    pair_gap: wp.array2d(dtype=float),
    pair_friction: wp.array2d(dtype=vec5),
    # Data in:
    naconmax_in: int,
    geom_xpos_in: wp.array2d(dtype=wp.vec3),
    geom_xmat_in: wp.array2d(dtype=wp.mat33),
    collision_pair_in: wp.array(dtype=wp.vec2i),
    collision_pairid_in: wp.array(dtype=int),
    collision_worldid_in: wp.array(dtype=int),
    ncollision_in: wp.array(dtype=int),
    # Data out:
    nacon_out: wp.array(dtype=int),
    contact_dist_out: wp.array(dtype=float),
    contact_pos_out: wp.array(dtype=wp.vec3),
    contact_frame_out: wp.array(dtype=wp.mat33),
    contact_includemargin_out: wp.array(dtype=float),
    contact_friction_out: wp.array(dtype=vec5),
    contact_solref_out: wp.array(dtype=wp.vec2),
    contact_solreffriction_out: wp.array(dtype=wp.vec2),
    contact_solimp_out: wp.array(dtype=vec5),
    contact_dim_out: wp.array(dtype=int),
    contact_geom_out: wp.array(dtype=wp.vec2i),
    contact_worldid_out: wp.array(dtype=int),
  ):
    tid = wp.tid()

    if tid >= ncollision_in[0]:
      return

    geoms = collision_pair_in[tid]
    g1 = geoms[0]
    g2 = geoms[1]

    type1 = geom_type[g1]
    type2 = geom_type[g2]

    worldid = collision_worldid_in[tid]

    _, margin, gap, condim, friction, solref, solreffriction, solimp = contact_params(
      geom_condim,
      geom_priority,
      geom_solmix,
      geom_solref,
      geom_solimp,
      geom_friction,
      geom_margin,
      geom_gap,
      pair_dim,
      pair_solref,
      pair_solreffriction,
      pair_solimp,
      pair_margin,
      pair_gap,
      pair_friction,
      collision_pair_in,
      collision_pairid_in,
      tid,
      worldid,
    )

    geom1_dataid = geom_dataid[g1]
    geom1 = geom(
      type1,
      geom1_dataid,
      geom_size[worldid, g1],
      mesh_vertadr,
      mesh_vertnum,
      mesh_vert,
      mesh_graphadr,
      mesh_graph,
      mesh_polynum,
      mesh_polyadr,
      mesh_polynormal,
      mesh_polyvertadr,
      mesh_polyvertnum,
      mesh_polyvert,
      mesh_polymapadr,
      mesh_polymapnum,
      mesh_polymap,
      geom_xpos_in[worldid, g1],
      geom_xmat_in[worldid, g1],
    )

    geom2_dataid = geom_dataid[g2]
    geom2 = geom(
      type2,
      geom2_dataid,
      geom_size[worldid, g2],
      mesh_vertadr,
      mesh_vertnum,
      mesh_vert,
      mesh_graphadr,
      mesh_graph,
      mesh_polynum,
      mesh_polyadr,
      mesh_polynormal,
      mesh_polyvertadr,
      mesh_polyvertnum,
      mesh_polyvert,
      mesh_polymapadr,
      mesh_polymapnum,
      mesh_polymap,
      geom_xpos_in[worldid, g2],
      geom_xmat_in[worldid, g2],
    )

    for i in range(wp.static(len(primitive_collisions_func))):
      collision_type1 = wp.static(primitive_collisions_types[i][0])
      collision_type2 = wp.static(primitive_collisions_types[i][1])

      if collision_type1 == type1 and collision_type2 == type2:
        wp.static(primitive_collisions_func[i])(
          naconmax_in,
          geom1,
          geom2,
          worldid,
          margin,
          gap,
          condim,
          friction,
          solref,
          solreffriction,
          solimp,
          geoms,
          nacon_out,
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
          contact_worldid_out,
        )

  return _primitive_narrowphase


def _primitive_narrowphase_builder(m: Model):
  _primitive_collisions_types = []
  _primitive_collisions_func = []

  for types, func in _PRIMITIVE_COLLISIONS.items():
    idx = upper_trid_index(len(GeomType), types[0].value, types[1].value)
    if m.geom_pair_type_count[idx] and types not in _primitive_collisions_types:
      _primitive_collisions_types.append(types)
      _primitive_collisions_func.append(func)

  return _create_narrowphase_kernel(_primitive_collisions_types, _primitive_collisions_func)


@event_scope
def primitive_narrowphase(m: Model, d: Data):
  """Runs collision detection on primitive geom pairs discovered during broadphase.

  This function processes collision pairs involving primitive shapes that were
  identified during the broadphase stage. It computes detailed contact information
  such as distance, position, and frame, and populates the `d.contact` array.

  The primitive geom types handled are PLANE, SPHERE, CAPSULE, CYLINDER, BOX.

  It also handles collisions between planes and convex hulls.

  To improve performance, it dynamically builds and launches a kernel tailored to
  the specific primitive collision types present in the model, avoiding
  unnecessary checks for non-existent collision pairs.
  """
  # we need to figure out how to keep the overhead of this small - not launching anything
  # for pair types without collisions, as well as updating the launch dimensions.
  wp.launch(
    _primitive_narrowphase_builder(m),
    dim=d.naconmax,
    inputs=[
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
      m.hfield_adr,
      m.hfield_nrow,
      m.hfield_ncol,
      m.hfield_size,
      m.hfield_data,
      m.mesh_vertadr,
      m.mesh_vertnum,
      m.mesh_vert,
      m.mesh_graphadr,
      m.mesh_graph,
      m.mesh_polynum,
      m.mesh_polyadr,
      m.mesh_polynormal,
      m.mesh_polyvertadr,
      m.mesh_polyvertnum,
      m.mesh_polyvert,
      m.mesh_polymapadr,
      m.mesh_polymapnum,
      m.mesh_polymap,
      m.pair_dim,
      m.pair_solref,
      m.pair_solreffriction,
      m.pair_solimp,
      m.pair_margin,
      m.pair_gap,
      m.pair_friction,
      d.naconmax,
      d.geom_xpos,
      d.geom_xmat,
      d.collision_pair,
      d.collision_pairid,
      d.collision_worldid,
      d.ncollision,
    ],
    outputs=[
      d.nacon,
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
      d.contact.worldid,
    ],
  )
