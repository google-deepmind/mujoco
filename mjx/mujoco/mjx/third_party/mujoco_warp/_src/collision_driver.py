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

from typing import Any

import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src.collision_convex import convex_narrowphase
from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive import primitive_narrowphase
from mujoco.mjx.third_party.mujoco_warp._src.collision_sdf import sdf_narrowphase
from mujoco.mjx.third_party.mujoco_warp._src.math import upper_tri_index
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MAXVAL
from mujoco.mjx.third_party.mujoco_warp._src.types import BroadphaseFilter
from mujoco.mjx.third_party.mujoco_warp._src.types import BroadphaseType
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import DisableBit
from mujoco.mjx.third_party.mujoco_warp._src.types import Model
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import cache_kernel
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import kernel as nested_kernel

wp.set_module_options({"enable_backward": False})


@wp.kernel
def _zero_nacon_ncollision(
  # Data out:
  nacon_out: wp.array(dtype=int),
  ncollision_out: wp.array(dtype=int),
):
  ncollision_out[0] = 0
  nacon_out[0] = 0


@wp.func
def _plane_filter(
  size1: float, size2: float, margin1: float, margin2: float, xpos1: wp.vec3, xpos2: wp.vec3, xmat1: wp.mat33, xmat2: wp.mat33
) -> bool:
  if size1 == 0.0:
    # geom1 is a plane
    dist = wp.dot(xpos2 - xpos1, wp.vec3(xmat1[0, 2], xmat1[1, 2], xmat1[2, 2]))
    return dist <= size2 + wp.max(margin1, margin2)
  elif size2 == 0.0:
    # geom2 is a plane
    dist = wp.dot(xpos1 - xpos2, wp.vec3(xmat2[0, 2], xmat2[1, 2], xmat2[2, 2]))
    return dist <= size1 + wp.max(margin1, margin2)

  return True


@wp.func
def _sphere_filter(size1: float, size2: float, margin1: float, margin2: float, xpos1: wp.vec3, xpos2: wp.vec3) -> bool:
  bound = size1 + size2 + wp.max(margin1, margin2)
  dif = xpos2 - xpos1
  dist_sq = wp.dot(dif, dif)
  return dist_sq <= bound * bound


# TODO(team): improve performance by precomputing bounding box
@wp.func
def _aabb_filter(
  # In:
  center1: wp.vec3,
  center2: wp.vec3,
  size1: wp.vec3,
  size2: wp.vec3,
  margin1: float,
  margin2: float,
  xpos1: wp.vec3,
  xpos2: wp.vec3,
  xmat1: wp.mat33,
  xmat2: wp.mat33,
) -> bool:
  """Axis aligned boxes collision.

  references: see Ericson, Real-time Collision Detection section 4.2.
              filterBox: filter contact based on global AABBs.
  """
  center1 = xmat1 @ center1 + xpos1
  center2 = xmat2 @ center2 + xpos2

  margin = wp.max(margin1, margin2)

  max_x1 = -MJ_MAXVAL
  max_y1 = -MJ_MAXVAL
  max_z1 = -MJ_MAXVAL
  min_x1 = MJ_MAXVAL
  min_y1 = MJ_MAXVAL
  min_z1 = MJ_MAXVAL

  max_x2 = -MJ_MAXVAL
  max_y2 = -MJ_MAXVAL
  max_z2 = -MJ_MAXVAL
  min_x2 = MJ_MAXVAL
  min_y2 = MJ_MAXVAL
  min_z2 = MJ_MAXVAL

  sign = wp.vec2(-1.0, 1.0)

  for i in range(2):
    for j in range(2):
      for k in range(2):
        corner1 = wp.vec3(sign[i] * size1[0], sign[j] * size1[1], sign[k] * size1[2])
        pos1 = xmat1 @ corner1

        corner2 = wp.vec3(sign[i] * size2[0], sign[j] * size2[1], sign[k] * size2[2])
        pos2 = xmat2 @ corner2

        if pos1[0] > max_x1:
          max_x1 = pos1[0]

        if pos1[1] > max_y1:
          max_y1 = pos1[1]

        if pos1[2] > max_z1:
          max_z1 = pos1[2]

        if pos1[0] < min_x1:
          min_x1 = pos1[0]

        if pos1[1] < min_y1:
          min_y1 = pos1[1]

        if pos1[2] < min_z1:
          min_z1 = pos1[2]

        if pos2[0] > max_x2:
          max_x2 = pos2[0]

        if pos2[1] > max_y2:
          max_y2 = pos2[1]

        if pos2[2] > max_z2:
          max_z2 = pos2[2]

        if pos2[0] < min_x2:
          min_x2 = pos2[0]

        if pos2[1] < min_y2:
          min_y2 = pos2[1]

        if pos2[2] < min_z2:
          min_z2 = pos2[2]

  if center1[0] + max_x1 + margin < center2[0] + min_x2:
    return False
  if center1[1] + max_y1 + margin < center2[1] + min_y2:
    return False
  if center1[2] + max_z1 + margin < center2[2] + min_z2:
    return False
  if center2[0] + max_x2 + margin < center1[0] + min_x1:
    return False
  if center2[1] + max_y2 + margin < center1[1] + min_y1:
    return False
  if center2[2] + max_z2 + margin < center1[2] + min_z1:
    return False

  return True


mat23 = wp.types.matrix(shape=(2, 3), dtype=float)
mat63 = wp.types.matrix(shape=(6, 3), dtype=float)


# TODO(team): improve performance by precomputing bounding box
@wp.func
def _obb_filter(
  # In:
  center1: wp.vec3,
  center2: wp.vec3,
  size1: wp.vec3,
  size2: wp.vec3,
  margin1: float,
  margin2: float,
  xpos1: wp.vec3,
  xpos2: wp.vec3,
  xmat1: wp.mat33,
  xmat2: wp.mat33,
) -> bool:
  """Oriented bounding boxes collision (see Gottschalk et al.), see mj_collideOBB."""
  margin = wp.max(margin1, margin2)

  xcenter = mat23()
  normal = mat63()
  proj = wp.vec2()
  radius = wp.vec2()

  # compute centers in local coordinates
  xcenter[0] = xmat1 @ center1 + xpos1
  xcenter[1] = xmat2 @ center2 + xpos2

  # compute normals in global coordinates
  normal[0] = wp.vec3(xmat1[0, 0], xmat1[1, 0], xmat1[2, 0])
  normal[1] = wp.vec3(xmat1[0, 1], xmat1[1, 1], xmat1[2, 1])
  normal[2] = wp.vec3(xmat1[0, 2], xmat1[1, 2], xmat1[2, 2])
  normal[3] = wp.vec3(xmat2[0, 0], xmat2[1, 0], xmat2[2, 0])
  normal[4] = wp.vec3(xmat2[0, 1], xmat2[1, 1], xmat2[2, 1])
  normal[5] = wp.vec3(xmat2[0, 2], xmat2[1, 2], xmat2[2, 2])

  # check intersections
  for j in range(2):
    for k in range(3):
      for i in range(2):
        proj[i] = wp.dot(xcenter[i], normal[3 * j + k])
        if i == 0:
          size = size1
        else:
          size = size2

        # fmt: off
        radius[i] = (
            wp.abs(size[0] * wp.dot(normal[3 * i + 0], normal[3 * j + k]))
          + wp.abs(size[1] * wp.dot(normal[3 * i + 1], normal[3 * j + k]))
          + wp.abs(size[2] * wp.dot(normal[3 * i + 2], normal[3 * j + k]))
        )
        # fmt: on
      if radius[0] + radius[1] + margin < wp.abs(proj[1] - proj[0]):
        return False

  return True


def _broadphase_filter(m: Model):
  @wp.func
  def func(
    # Model:
    geom_aabb: wp.array3d(dtype=wp.vec3),
    geom_rbound: wp.array2d(dtype=float),
    geom_margin: wp.array2d(dtype=float),
    # Data in:
    geom_xpos_in: wp.array2d(dtype=wp.vec3),
    geom_xmat_in: wp.array2d(dtype=wp.mat33),
    # In:
    geom1: int,
    geom2: int,
    worldid: int,
  ) -> bool:
    # 1: plane
    # 2: sphere
    # 4: aabb
    # 8: obb

    aabb_id = worldid % geom_aabb.shape[0] if wp.static(m.geom_aabb.shape[0] > 1) else 0
    center1, center2 = geom_aabb[aabb_id, geom1, 0], geom_aabb[aabb_id, geom2, 0]
    size1, size2 = geom_aabb[aabb_id, geom1, 1], geom_aabb[aabb_id, geom2, 1]

    rbound_id = worldid % geom_rbound.shape[0] if wp.static(m.geom_rbound.shape[0] > 1) else 0
    rbound1, rbound2 = geom_rbound[rbound_id, geom1], geom_rbound[rbound_id, geom2]
    margin_id = worldid % geom_margin.shape[0] if wp.static(m.geom_margin.shape[0] > 1) else 0
    margin1, margin2 = geom_margin[margin_id, geom1], geom_margin[margin_id, geom2]
    xpos1, xpos2 = geom_xpos_in[worldid, geom1], geom_xpos_in[worldid, geom2]
    xmat1, xmat2 = geom_xmat_in[worldid, geom1], geom_xmat_in[worldid, geom2]

    if rbound1 == 0.0 or rbound2 == 0.0:
      if wp.static(m.opt.broadphase_filter & BroadphaseFilter.PLANE):
        return _plane_filter(rbound1, rbound2, margin1, margin2, xpos1, xpos2, xmat1, xmat2)
    else:
      if wp.static(m.opt.broadphase_filter & BroadphaseFilter.SPHERE):
        if not _sphere_filter(rbound1, rbound2, margin1, margin2, xpos1, xpos2):
          return False
      if wp.static(m.opt.broadphase_filter & BroadphaseFilter.AABB):
        if not _aabb_filter(center1, center2, size1, size2, margin1, margin2, xpos1, xpos2, xmat1, xmat2):
          return False
      if wp.static(m.opt.broadphase_filter & BroadphaseFilter.OBB):
        if not _obb_filter(center1, center2, size1, size2, margin1, margin2, xpos1, xpos2, xmat1, xmat2):
          return False

    return True

  return func


@wp.func
def _add_geom_pair(
  # Model:
  geom_type: wp.array(dtype=int),
  nxn_pairid: wp.array(dtype=wp.vec2i),
  # Data in:
  naconmax_in: int,
  # In:
  geom1: int,
  geom2: int,
  worldid: int,
  nxnid: int,
  # Data out:
  collision_pair_out: wp.array(dtype=wp.vec2i),
  collision_pairid_out: wp.array(dtype=wp.vec2i),
  collision_worldid_out: wp.array(dtype=int),
  ncollision_out: wp.array(dtype=int),
):
  pairid = wp.atomic_add(ncollision_out, 0, 1)

  if pairid >= naconmax_in:
    return

  type1 = geom_type[geom1]
  type2 = geom_type[geom2]

  if type1 > type2:
    pair = wp.vec2i(geom2, geom1)
  else:
    pair = wp.vec2i(geom1, geom2)

  collision_pair_out[pairid] = pair
  collision_pairid_out[pairid] = nxn_pairid[nxnid]
  collision_worldid_out[pairid] = worldid


@wp.func
def _binary_search(values: wp.array(dtype=Any), value: Any, lower: int, upper: int) -> int:
  while lower < upper:
    mid = (lower + upper) >> 1
    if values[mid] > value:
      upper = mid
    else:
      lower = mid + 1

  return upper


def _sap_project(opt_broadphase: int):
  @nested_kernel(module="unique", enable_backward=False)
  def sap_project(
    # Model:
    ngeom: int,
    geom_rbound: wp.array2d(dtype=float),
    geom_margin: wp.array2d(dtype=float),
    # Data in:
    nworld_in: int,
    geom_xpos_in: wp.array2d(dtype=wp.vec3),
    # In:
    direction_in: wp.vec3,
    # Out:
    projection_lower_out: wp.array2d(dtype=float),
    projection_upper_out: wp.array2d(dtype=float),
    sort_index_out: wp.array2d(dtype=int),
    segmented_index_out: wp.array(dtype=int),
  ):
    worldid, geomid = wp.tid()

    xpos = geom_xpos_in[worldid, geomid]
    rbound = geom_rbound[worldid % geom_rbound.shape[0], geomid]

    if rbound == 0.0:
      # geom is a plane
      rbound = MJ_MAXVAL

    radius = rbound + geom_margin[worldid % geom_margin.shape[0], geomid]
    center = wp.dot(direction_in, xpos)

    sort_index_out[worldid, geomid] = geomid
    if not wp.isnan(center):
      projection_lower_out[worldid, geomid] = center - radius
      projection_upper_out[worldid, geomid] = center + radius
    else:
      projection_lower_out[worldid, geomid] = MJ_MAXVAL
      projection_upper_out[worldid, geomid] = MJ_MAXVAL

    if wp.static(opt_broadphase == BroadphaseType.SAP_SEGMENTED):
      if geomid == 0:
        segmented_index_out[worldid] = worldid * ngeom
        if worldid == nworld_in - 1:
          segmented_index_out[nworld_in] = nworld_in * ngeom

  return sap_project


@wp.kernel
def _sap_range(
  # Model:
  ngeom: int,
  # In:
  projection_lower_in: wp.array2d(dtype=float),
  projection_upper_in: wp.array2d(dtype=float),
  sort_index_in: wp.array2d(dtype=int),
  # Out:
  range_out: wp.array2d(dtype=int),
):
  worldid, geomid = wp.tid()

  # current bounding geom
  idx = sort_index_in[worldid, geomid]

  upper = projection_upper_in[worldid, idx]

  limit = _binary_search(projection_lower_in[worldid], upper, geomid + 1, ngeom)
  limit = wp.min(ngeom - 1, limit)

  # range of geoms for the sweep and prune process
  range_out[worldid, geomid] = limit - geomid


@cache_kernel
def _sap_broadphase(broadphase_filter):
  @nested_kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    ngeom: int,
    geom_type: wp.array(dtype=int),
    geom_aabb: wp.array3d(dtype=wp.vec3),
    geom_rbound: wp.array2d(dtype=float),
    geom_margin: wp.array2d(dtype=float),
    nxn_pairid: wp.array(dtype=wp.vec2i),
    # Data in:
    nworld_in: int,
    naconmax_in: int,
    geom_xpos_in: wp.array2d(dtype=wp.vec3),
    geom_xmat_in: wp.array2d(dtype=wp.mat33),
    # In:
    sort_index_in: wp.array2d(dtype=int),
    cumulative_sum_in: wp.array(dtype=int),
    nsweep_in: int,
    # Data out:
    collision_pair_out: wp.array(dtype=wp.vec2i),
    collision_pairid_out: wp.array(dtype=wp.vec2i),
    collision_worldid_out: wp.array(dtype=int),
    ncollision_out: wp.array(dtype=int),
  ):
    worldgeomid = wp.tid()

    nworldgeom = nworld_in * ngeom
    nworkpackages = cumulative_sum_in[nworldgeom - 1]

    while worldgeomid < nworkpackages:
      # binary search to find current and next geom pair indices
      i = _binary_search(cumulative_sum_in, worldgeomid, 0, nworldgeom)
      j = i + worldgeomid + 1

      if i > 0:
        j -= cumulative_sum_in[i - 1]

      worldid = i // ngeom
      i = i % ngeom
      j = j % ngeom

      # get geom indices and swap if necessary
      geom1 = sort_index_in[worldid, i]
      geom2 = sort_index_in[worldid, j]

      # find linear index of (geom1, geom2) in upper triangular nxn_pairid
      if geom2 < geom1:
        idx = upper_tri_index(ngeom, geom2, geom1)
      else:
        idx = upper_tri_index(ngeom, geom1, geom2)

      worldgeomid += nsweep_in
      pairid = nxn_pairid[idx]
      if pairid[0] < -1 and pairid[1] < 0:
        continue

      if (
        broadphase_filter(geom_aabb, geom_rbound, geom_margin, geom_xpos_in, geom_xmat_in, geom1, geom2, worldid)
        or pairid[1] >= 0
      ):
        _add_geom_pair(
          geom_type,
          nxn_pairid,
          naconmax_in,
          geom1,
          geom2,
          worldid,
          idx,
          collision_pair_out,
          collision_pairid_out,
          collision_worldid_out,
          ncollision_out,
        )

  return kernel


def _segmented_sort(tile_size: int):
  @wp.kernel
  def segmented_sort(
    # In:
    projection_lower_in: wp.array2d(dtype=float),
    sort_index_in: wp.array2d(dtype=int),
    # Out:
    projection_lower_out: wp.array2d(dtype=float),
    sort_index_out: wp.array2d(dtype=int),
  ):
    worldid = wp.tid()

    # Load input into shared memory
    keys = wp.tile_load(projection_lower_in[worldid], shape=tile_size, storage="shared")
    values = wp.tile_load(sort_index_in[worldid], shape=tile_size, storage="shared")

    # Perform in-place sorting
    wp.tile_sort(keys, values)

    # Store sorted shared memory into output arrays
    wp.tile_store(projection_lower_out[worldid], keys)
    wp.tile_store(sort_index_out[worldid], values)

  return segmented_sort


@event_scope
def sap_broadphase(m: Model, d: Data):
  """Runs broadphase collision detection using a sweep-and-prune (SAP) algorithm.

  This method is more efficient than the N-squared approach for large numbers of
  objects. It works by projecting the bounding spheres of all geoms onto a
  single axis and sorting them. It then sweeps along the axis, only checking
  for overlaps between geoms whose projections are close to each other.

  For each potentially colliding pair identified by the sweep, a more precise
  bounding sphere check is performed. If this check passes, the pair is added
  to the collision arrays in `d` for the narrowphase stage.

  Two sorting strategies are supported, controlled by `m.opt.broadphase`

  - `SAP_TILE`: Uses a tile-based sort.
  - `SAP_SEGMENTED`: Uses a segmented sort.
  """
  nworldgeom = d.nworld * m.ngeom

  # TODO(team): direction

  # random fixed direction
  direction = wp.vec3(0.5935, 0.7790, 0.1235)
  direction = wp.normalize(direction)

  projection_lower = wp.empty((d.nworld, m.ngeom, 2), dtype=float)
  projection_upper = wp.empty((d.nworld, m.ngeom), dtype=float)
  sort_index = wp.empty((d.nworld, m.ngeom, 2), dtype=int)
  range_ = wp.empty((d.nworld, m.ngeom), dtype=int)
  cumulative_sum = wp.empty((d.nworld, m.ngeom), dtype=int)
  segmented_index = wp.empty(d.nworld + 1 if m.opt.broadphase == BroadphaseType.SAP_SEGMENTED else 0, dtype=int)

  wp.launch(
    kernel=_sap_project(m.opt.broadphase),
    dim=(d.nworld, m.ngeom),
    inputs=[m.ngeom, m.geom_rbound, m.geom_margin, d.nworld, d.geom_xpos, direction],
    outputs=[
      projection_lower.reshape((-1, m.ngeom)),
      projection_upper,
      sort_index.reshape((-1, m.ngeom)),
      segmented_index,
    ],
  )

  if m.opt.broadphase == BroadphaseType.SAP_TILE:
    wp.launch_tiled(
      kernel=_segmented_sort(m.ngeom),
      dim=d.nworld,
      inputs=[projection_lower.reshape((-1, m.ngeom)), sort_index.reshape((-1, m.ngeom))],
      outputs=[projection_lower.reshape((-1, m.ngeom)), sort_index.reshape((-1, m.ngeom))],
      block_dim=m.block_dim.segmented_sort,
    )
  else:
    wp.utils.segmented_sort_pairs(
      projection_lower.reshape((-1, m.ngeom)), sort_index.reshape((-1, m.ngeom)), nworldgeom, segmented_index
    )

  wp.launch(
    kernel=_sap_range,
    dim=(d.nworld, m.ngeom),
    inputs=[m.ngeom, projection_lower.reshape((-1, m.ngeom)), projection_upper, sort_index.reshape((-1, m.ngeom))],
    outputs=[range_],
  )

  # scan is used for load balancing among the threads
  wp.utils.array_scan(range_.reshape(-1), cumulative_sum.reshape(-1), True)

  # estimate number of overlap checks
  # assumes each geom has 5 other geoms (batched over all worlds)
  nsweep = 5 * nworldgeom
  broadphase_filter = _broadphase_filter(m)
  wp.launch(
    kernel=_sap_broadphase(broadphase_filter),
    dim=nsweep,
    inputs=[
      m.ngeom,
      m.geom_type,
      m.geom_aabb,
      m.geom_rbound,
      m.geom_margin,
      m.nxn_pairid,
      d.nworld,
      d.naconmax,
      d.geom_xpos,
      d.geom_xmat,
      sort_index.reshape((-1, m.ngeom)),
      cumulative_sum.reshape(-1),
      nsweep,
    ],
    outputs=[d.collision_pair, d.collision_pairid, d.collision_worldid, d.ncollision],
  )


@cache_kernel
def _nxn_broadphase(broadphase_filter):
  @nested_kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    geom_type: wp.array(dtype=int),
    geom_aabb: wp.array3d(dtype=wp.vec3),
    geom_rbound: wp.array2d(dtype=float),
    geom_margin: wp.array2d(dtype=float),
    nxn_geom_pair: wp.array(dtype=wp.vec2i),
    nxn_pairid: wp.array(dtype=wp.vec2i),
    # Data in:
    naconmax_in: int,
    geom_xpos_in: wp.array2d(dtype=wp.vec3),
    geom_xmat_in: wp.array2d(dtype=wp.mat33),
    # Data out:
    collision_pair_out: wp.array(dtype=wp.vec2i),
    collision_pairid_out: wp.array(dtype=wp.vec2i),
    collision_worldid_out: wp.array(dtype=int),
    ncollision_out: wp.array(dtype=int),
  ):
    worldid, elementid = wp.tid()

    geom = nxn_geom_pair[elementid]
    geom1 = geom[0]
    geom2 = geom[1]

    if (
      broadphase_filter(geom_aabb, geom_rbound, geom_margin, geom_xpos_in, geom_xmat_in, geom1, geom2, worldid)
      or nxn_pairid[elementid][1] >= 0
    ):
      _add_geom_pair(
        geom_type,
        nxn_pairid,
        naconmax_in,
        geom1,
        geom2,
        worldid,
        elementid,
        collision_pair_out,
        collision_pairid_out,
        collision_worldid_out,
        ncollision_out,
      )

  return kernel


@event_scope
def nxn_broadphase(m: Model, d: Data):
  """Runs broadphase collision detection using a brute-force N-squared approach.

  This function iterates through a pre-filtered list of all possible geometry pairs and
  performs a quick bounding sphere check to identify potential collisions.

  For each pair that passes the sphere check, it populates the collision arrays in `d`
  (`d.collision_pair`, `d.collision_pairid`, etc.), which are then consumed by the
  narrowphase.

  The initial list of pairs is filtered at model creation time to exclude pairs based on
  `contype`/`conaffinity`, parent-child relationships, and explicit `<exclude>` tags.
  """
  broadphase_filter = _broadphase_filter(m)
  wp.launch(
    _nxn_broadphase(broadphase_filter),
    dim=(d.nworld, m.nxn_geom_pair_filtered.shape[0]),
    inputs=[
      m.geom_type,
      m.geom_aabb,
      m.geom_rbound,
      m.geom_margin,
      m.nxn_geom_pair_filtered,
      m.nxn_pairid_filtered,
      d.naconmax,
      d.geom_xpos,
      d.geom_xmat,
    ],
    outputs=[
      d.collision_pair,
      d.collision_pairid,
      d.collision_worldid,
      d.ncollision,
    ],
  )


def _narrowphase(m, d):
  # TODO(team): we should reject far-away contacts in the narrowphase instead of constraint
  #             partitioning because we can move some pressure of the atomics
  convex_narrowphase(m, d)
  primitive_narrowphase(m, d)

  if m.has_sdf_geom:
    sdf_narrowphase(m, d)


@event_scope
def collision(m: Model, d: Data):
  """Runs the full collision detection pipeline.

  This function orchestrates the broadphase and narrowphase collision detection stages. It
  first identifies potential collision pairs using a broadphase algorithm (either N-squared
  or Sweep-and-Prune, based on `m.opt.broadphase`). Then, for each potential pair, it
  performs narrowphase collision detection to compute detailed contact information like
  distance, position, and frame.

  The results are used to populate the `d.contact` array, and the total number of contacts
  is stored in `d.nacon`.  If `d.nacon` is larger than `d.naconmax` then an overflow has
  occurred and the remaining contacts will be skipped.  If this happens, raise the `nconmax`
  parameter in `io.make_data` or `io.put_data`.

  This function will do nothing except zero out arrays if collision detection is disabled
  via `m.opt.disableflags` or if `d.nacon` is 0.
  """
  # zero contact and collision counters
  wp.launch(_zero_nacon_ncollision, dim=1, outputs=[d.nacon, d.ncollision])

  if d.naconmax == 0 or m.opt.disableflags & (DisableBit.CONSTRAINT | DisableBit.CONTACT):
    return

  if m.opt.broadphase == BroadphaseType.NXN:
    nxn_broadphase(m, d)
  else:
    sap_broadphase(m, d)

  _narrowphase(m, d)
