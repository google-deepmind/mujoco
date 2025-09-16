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

from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MAXVAL
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import GeomType
from mujoco.mjx.third_party.mujoco_warp._src.types import Model


@wp.func
def _hfield_subgrid(
  # In:
  nrow: int,
  ncol: int,
  size: wp.vec4,
  xmax: float,
  xmin: float,
  ymax: float,
  ymin: float,
) -> Tuple[int, int, int, int]:
  """Returns height field subgrid that overlaps with geom AABB.

  Args:
    nrow: height field number of rows
    ncol: height field number of columns
    size: height field size
    xmax: geom maximum x position
    xmin: geom minimum x position
    ymax: geom maximum y position
    ymin: geom minimum y position

  Returns:
    grid coordinate bounds
  """

  # grid resolution
  x_scale = 0.5 * float(ncol - 1) / size[0]
  y_scale = 0.5 * float(nrow - 1) / size[1]

  # subgrid
  cmin = wp.max(0, int(wp.floor((xmin + size[0]) * x_scale)))
  cmax = wp.min(ncol - 1, int(wp.ceil((xmax + size[0]) * x_scale)))
  rmin = wp.max(0, int(wp.floor((ymin + size[1]) * y_scale)))
  rmax = wp.min(nrow - 1, int(wp.ceil((ymax + size[1]) * y_scale)))

  return cmin, rmin, cmax, rmax


@wp.func
def hfield_triangle_prism(
  # Model:
  geom_dataid: wp.array(dtype=int),
  hfield_adr: wp.array(dtype=int),
  hfield_nrow: wp.array(dtype=int),
  hfield_ncol: wp.array(dtype=int),
  hfield_size: wp.array(dtype=wp.vec4),
  hfield_data: wp.array(dtype=float),
  # In:
  hfieldid: int,
  hftri_index: int,
) -> wp.mat33:
  """Returns triangular prism vertex information in compressed representation.

  Args:
    geom_dataid: geom data ids
    hfield_adr: address for height field
    hfield_nrow: height field number of rows
    hfield_ncol: height field number of columns
    hfield_size: height field sizes
    hfield_data: height field data
    hfieldid: height field geom id
    hftri_index: height field triangle index

  Returns:
    triangular prism vertex information (compressed)
  """
  # https://mujoco.readthedocs.io/en/stable/XMLreference.html#asset-hfield

  # get heightfield dimensions
  dataid = geom_dataid[hfieldid]
  if dataid < 0 or hftri_index < 0:
    return wp.mat33(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

  nrow = hfield_nrow[dataid]
  ncol = hfield_ncol[dataid]
  size = hfield_size[dataid]  # (x, y, z_top, z_bottom)

  # calculate which triangle in the grid
  row = (hftri_index // 2) // (ncol - 1)
  col = (hftri_index // 2) % (ncol - 1)

  # calculate vertices in 2D grid
  x_scale = 2.0 * size[0] / float(ncol - 1)
  y_scale = 2.0 * size[1] / float(nrow - 1)

  # grid coordinates (i, j) for triangle corners
  i0 = col
  j0 = row
  i1 = i0 + 1
  j1 = j0 + 1

  # convert grid coordinates to local space x, y coordinates
  x0 = float(i0) * x_scale - size[0]
  y0 = float(j0) * y_scale - size[1]
  x1 = float(i1) * x_scale - size[0]
  y1 = float(j1) * y_scale - size[1]

  # get height values at corners from hfield_data
  base_addr = hfield_adr[dataid]
  z00 = hfield_data[base_addr + j0 * ncol + i0]
  z01 = hfield_data[base_addr + j1 * ncol + i0]
  z10 = hfield_data[base_addr + j0 * ncol + i1]
  z11 = hfield_data[base_addr + j1 * ncol + i1]

  # scale heights from range [0, 1] to [0, z_top]
  z_top = size[2]
  z00 = z00 * z_top
  z01 = z01 * z_top
  z10 = z10 * z_top
  z11 = z11 * z_top

  x2 = wp.where(hftri_index % 2, 1.0, 0.0)
  y2 = wp.where(hftri_index % 2, z10, z01)
  z22 = -size[3]

  # compress 6 prism vertices into 3x3 matrix, see hfield_prism_vertex for details
  return wp.mat33(x0, y0, z00,
                  x1, y1, z11,
                  x2, y2, z22)  # fmt: off


@wp.func
def hfield_prism_vertex(prism: wp.mat33, vert_index: int) -> wp.vec3:
  """Extracts vertices from a compressed triangular prism representation.

  The compression scheme stores a 6-vertex triangular prism using a 3x3 matrix:
  - prism[0] = First vertex (x,y,z) - corner (i,j)
  - prism[1] = Second vertex (x,y,z) - corner (i+1,j+1)
  - prism[2,0] = Triangle type flag: 0 for even triangle (using corner (i,j+1)),
                 non-zero for odd triangle (using corner (i+1,j))
  - prism[2,1] = Z-coordinate of the third vertex
  - prism[2,2] = Z-coordinate used for all bottom vertices (common z)

  In this way, we can reconstruct all 6 vertices of the prism by reusing
  coordinates from the stored vertices.

  Args:
      prism: 3x3 compressed representation of a triangular prism
      vert_index: index of vertex to extract (0-5)

  Returns:
      3D coordinates of the requested vertex
  """
  if vert_index == 0 or vert_index == 1:
    return prism[vert_index]  # first two vertices stored directly

  if vert_index == 2:  # third vertex
    if prism[2][0] == 0:  # even triangle (i, j+1)
      return wp.vec3(prism[0][0], prism[1][1], prism[2][1])
    else:  # odd triangle (i+1, j)
      return wp.vec3(prism[1][0], prism[0][1], prism[2][1])

  if vert_index == 3 or vert_index == 4:  # bottom vertices below 0 and 1
    return wp.vec3(prism[vert_index - 3][0], prism[vert_index - 3][1], prism[2][2])

  if vert_index == 5:  # bottom vertex below 2
    if prism[2][0] == 0:  # even triangle
      return wp.vec3(prism[0][0], prism[1][1], prism[2][2])
    else:  # odd triangle
      return wp.vec3(prism[1][0], prism[0][1], prism[2][2])


@wp.kernel
def _hfield_midphase(
  # Model:
  geom_type: wp.array(dtype=int),
  geom_dataid: wp.array(dtype=int),
  geom_aabb: wp.array2d(dtype=wp.vec3),
  geom_rbound: wp.array2d(dtype=float),
  geom_margin: wp.array2d(dtype=float),
  hfield_nrow: wp.array(dtype=int),
  hfield_ncol: wp.array(dtype=int),
  hfield_size: wp.array(dtype=wp.vec4),
  # Data in:
  nconmax_in: int,
  geom_xpos_in: wp.array2d(dtype=wp.vec3),
  geom_xmat_in: wp.array2d(dtype=wp.mat33),
  collision_pair_in: wp.array(dtype=wp.vec2i),
  collision_hftri_index_in: wp.array(dtype=int),
  collision_pairid_in: wp.array(dtype=int),
  collision_worldid_in: wp.array(dtype=int),
  # Data out:
  collision_pair_out: wp.array(dtype=wp.vec2i),
  collision_hftri_index_out: wp.array(dtype=int),
  collision_pairid_out: wp.array(dtype=int),
  collision_worldid_out: wp.array(dtype=int),
  ncollision_out: wp.array(dtype=int),
):
  """Midphase collision detection for heightfield triangles with other geoms.

  This kernel processes collision pairs where one geom is a heightfield (identified by
  collision_hftri_index_in[pairid] == -1) and expands them into multiple collision pairs,
  one for each potentially colliding triangle.

  Args:
    geom_type: geom type
    geom_dataid: geom data id
    geom_rbound: geom bounding sphere radius
    geom_margin: geom margin
    hfield_nrow: height field number of rows
    hfield_ncol: height field number of columns
    hfield_size: height field size
    nconmax_in: maximum number of contacts
    geom_xpos_in: geom position
    geom_xmat_in: geom orientation
    collision_pair_in: collision pair
    collision_hftri_index_in: triangle indices, -1 for height field pair
    collision_pairid_in: collision pair id from broadphase
    collision_worldid_in: collision world id from broadphase
    collision_pair_out: collision pair from midphase
    collision_hftri_index_out: triangle indices from midphase
    collision_pairid_out: collision pair id from midphase
    collision_worldid_out: collision world id from midphase
    ncollision_out: number of collisions from broadphase and midphase
  """
  pairid = wp.tid()

  # only process pairs that are marked for height field collision (-1)
  if collision_hftri_index_in[pairid] != -1:
    return

  # collision pair info
  worldid = collision_worldid_in[pairid]
  pair_id = collision_pairid_in[pairid]

  pair = collision_pair_in[pairid]
  g1 = pair[0]
  g2 = pair[1]

  hfieldid = g1
  geomid = g2

  # SHOULD NOT OCCUR: if the first geom is not a heightfield, swap
  if geom_type[g1] != int(GeomType.HFIELD.value):
    hfieldid = g2
    geomid = g1

  # height field info
  hfdataid = geom_dataid[hfieldid]
  size1 = hfield_size[hfdataid]
  pos1 = geom_xpos_in[worldid, hfieldid]
  mat1 = geom_xmat_in[worldid, hfieldid]
  mat1T = wp.transpose(mat1)

  # geom info
  pos2 = geom_xpos_in[worldid, geomid]
  pos = mat1T @ (pos2 - pos1)
  r2 = geom_rbound[worldid, geomid]

  # TODO(team): margin?
  margin = wp.max(geom_margin[worldid, hfieldid], geom_margin[worldid, geomid])

  # box-sphere test: horizontal plane
  for i in range(2):
    if (size1[i] < pos[i] - r2 - margin) or (-size1[i] > pos[i] + r2 + margin):
      return

  # box-sphere test: vertical direction
  if size1[2] < pos[2] - r2 - margin:  # up
    return

  if -size1[3] > pos[2] + r2 + margin:  # down
    return

  mat2 = geom_xmat_in[worldid, geomid]
  mat = mat1T @ mat2

  # aabb for geom in height field frame
  xmax = -MJ_MAXVAL
  ymax = -MJ_MAXVAL
  zmax = -MJ_MAXVAL
  xmin = MJ_MAXVAL
  ymin = MJ_MAXVAL
  zmin = MJ_MAXVAL

  center2 = geom_aabb[geomid, 0]
  size2 = geom_aabb[geomid, 1]

  pos += mat1T @ center2

  sign = wp.vec2(-1.0, 1.0)

  for i in range(2):
    for j in range(2):
      for k in range(2):
        corner_local = wp.vec3(sign[i] * size2[0], sign[j] * size2[1], sign[k] * size2[2])
        corner_hf = mat @ corner_local

        if corner_hf[0] > xmax:
          xmax = corner_hf[0]
        if corner_hf[1] > ymax:
          ymax = corner_hf[1]
        if corner_hf[2] > zmax:
          zmax = corner_hf[2]
        if corner_hf[0] < xmin:
          xmin = corner_hf[0]
        if corner_hf[1] < ymin:
          ymin = corner_hf[1]
        if corner_hf[2] < zmin:
          zmin = corner_hf[2]

  xmax += pos[0]
  xmin += pos[0]
  ymax += pos[1]
  ymin += pos[1]
  zmax += pos[2]
  zmin += pos[2]

  # box-box test
  if (
    (xmin - margin > size1[0])
    or (xmax + margin < -size1[0])
    or (ymin - margin > size1[1])
    or (ymax + margin < -size1[1])
    or (zmin - margin > size1[2])
    or (zmax + margin < -size1[3])
  ):
    return

  # height field subgrid
  nrow = hfield_nrow[hfieldid]
  ncol = hfield_ncol[hfieldid]
  size = hfield_size[hfieldid]
  cmin, rmin, cmax, rmax = _hfield_subgrid(nrow, ncol, size, xmax, xmin, ymax, ymin)

  # loop over subgrid triangles
  for r in range(rmin, rmax):
    for c in range(cmin, cmax):
      # add both triangles from this cell
      for i in range(2):
        if r == rmin and c == cmin and i == 0:
          # reuse the initial pair for the 1st triangle
          new_pairid = pairid
        else:
          # create a new pair
          new_pairid = wp.atomic_add(ncollision_out, 0, 1)

        if new_pairid >= nconmax_in:
          return

        collision_pair_out[new_pairid] = pair
        collision_hftri_index_out[new_pairid] = 2 * (r * (ncol - 1) + c) + i
        collision_pairid_out[new_pairid] = pair_id
        collision_worldid_out[new_pairid] = worldid


def hfield_midphase(m: Model, d: Data):
  """Midphase collision detection for height field triangles with other geoms.

  Processes collision pairs from the broadphase where one geom is a height field and expands
  them into multiple collision pairs, one for each potentially colliding triangle. The
  function directly writes to the same collision buffers used by _add_geom_pair.
  """
  wp.launch(
    kernel=_hfield_midphase,
    dim=d.nconmax,
    inputs=[
      m.geom_type,
      m.geom_dataid,
      m.geom_aabb,
      m.geom_rbound,
      m.geom_margin,
      m.hfield_nrow,
      m.hfield_ncol,
      m.hfield_size,
      d.nconmax,
      d.geom_xpos,
      d.geom_xmat,
      d.collision_pair,
      d.collision_hftri_index,
      d.collision_pairid,
      d.collision_worldid,
    ],
    outputs=[d.collision_pair, d.collision_hftri_index, d.collision_pairid, d.collision_worldid, d.ncollision],
  )
