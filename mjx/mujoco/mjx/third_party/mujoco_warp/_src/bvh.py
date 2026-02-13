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
# ==============================================================================

from __future__ import annotations

from typing import Tuple

import mujoco
import numpy as np
import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MAXVAL
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import GeomType
from mujoco.mjx.third_party.mujoco_warp._src.types import Model
from mujoco.mjx.third_party.mujoco_warp._src.types import RenderContext
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope

wp.set_module_options({"enable_backward": False})


@event_scope
def refit_bvh(m: Model, d: Data, rc: RenderContext):
  """Refit the dynamic BVH structures in the render context."""
  refit_scene_bvh(m, d, rc)
  if m.nflex:
    refit_flex_bvh(m, d, rc)


@wp.func
def _compute_box_bounds(
  # In:
  pos: wp.vec3,
  rot: wp.mat33,
  size: wp.vec3,
) -> Tuple[wp.vec3, wp.vec3]:
  min_bound = wp.vec3(MJ_MAXVAL, MJ_MAXVAL, MJ_MAXVAL)
  max_bound = wp.vec3(-MJ_MAXVAL, -MJ_MAXVAL, -MJ_MAXVAL)

  for i in range(2):
    for j in range(2):
      for k in range(2):
        local_corner = wp.vec3(
          size[0] * (2.0 * float(i) - 1.0),
          size[1] * (2.0 * float(j) - 1.0),
          size[2] * (2.0 * float(k) - 1.0),
        )
        world_corner = pos + rot @ local_corner
        min_bound = wp.min(min_bound, world_corner)
        max_bound = wp.max(max_bound, world_corner)

  return min_bound, max_bound


@wp.func
def _compute_sphere_bounds(
  # In:
  pos: wp.vec3,
  rot: wp.mat33,
  size: wp.vec3,
) -> Tuple[wp.vec3, wp.vec3]:
  radius = size[0]
  return pos - wp.vec3(radius, radius, radius), pos + wp.vec3(radius, radius, radius)


@wp.func
def _compute_capsule_bounds(
  # In:
  pos: wp.vec3,
  rot: wp.mat33,
  size: wp.vec3,
) -> Tuple[wp.vec3, wp.vec3]:
  radius = size[0]
  half_length = size[1]
  z = wp.vec3(rot[0, 2], rot[1, 2], rot[2, 2])
  world_end1 = pos - z * half_length
  world_end2 = pos + z * half_length

  seg_min = wp.min(world_end1, world_end2)
  seg_max = wp.max(world_end1, world_end2)

  inflate = wp.vec3(radius, radius, radius)
  return seg_min - inflate, seg_max + inflate


@wp.func
def _compute_plane_bounds(
  # In:
  pos: wp.vec3,
  rot: wp.mat33,
  size: wp.vec3,
) -> Tuple[wp.vec3, wp.vec3]:
  # If plane size is non-positive, treat as infinite plane and use a large default extent
  size_scale = wp.max(size[0], size[1]) * 2.0
  if size[0] <= 0.0 or size[1] <= 0.0:
    size_scale = 1000.0
  min_bound = wp.vec3(MJ_MAXVAL, MJ_MAXVAL, MJ_MAXVAL)
  max_bound = wp.vec3(-MJ_MAXVAL, -MJ_MAXVAL, -MJ_MAXVAL)

  for i in range(2):
    for j in range(2):
      local_corner = wp.vec3(
        size_scale * (2.0 * float(i) - 1.0),
        size_scale * (2.0 * float(j) - 1.0),
        0.0,
      )
      world_corner = pos + rot @ local_corner
      min_bound = wp.min(min_bound, world_corner)
      max_bound = wp.max(max_bound, world_corner)

  min_bound = min_bound - wp.vec3(0.01, 0.01, 0.01)
  max_bound = max_bound + wp.vec3(0.01, 0.01, 0.01)

  return min_bound, max_bound


@wp.func
def _compute_ellipsoid_bounds(
  # In:
  pos: wp.vec3,
  rot: wp.mat33,
  size: wp.vec3,
) -> Tuple[wp.vec3, wp.vec3]:
  # Half-extent along each world axis equals the norm of the corresponding row of rot*diag(size)
  row0 = wp.vec3(rot[0, 0] * size[0], rot[0, 1] * size[1], rot[0, 2] * size[2])
  row1 = wp.vec3(rot[1, 0] * size[0], rot[1, 1] * size[1], rot[1, 2] * size[2])
  row2 = wp.vec3(rot[2, 0] * size[0], rot[2, 1] * size[1], rot[2, 2] * size[2])
  extent = wp.vec3(wp.length(row0), wp.length(row1), wp.length(row2))
  return pos - extent, pos + extent


@wp.func
def _compute_cylinder_bounds(
  # In:
  pos: wp.vec3,
  rot: wp.mat33,
  size: wp.vec3,
) -> Tuple[wp.vec3, wp.vec3]:
  radius = size[0]
  half_height = size[1]

  axis = wp.vec3(rot[0, 2], rot[1, 2], rot[2, 2])
  axis_abs = wp.vec3(wp.abs(axis[0]), wp.abs(axis[1]), wp.abs(axis[2]))

  basis_x = wp.vec3(rot[0, 0], rot[1, 0], rot[2, 0])
  basis_y = wp.vec3(rot[0, 1], rot[1, 1], rot[2, 1])

  radial_x = radius * wp.sqrt(basis_x[0] * basis_x[0] + basis_y[0] * basis_y[0])
  radial_y = radius * wp.sqrt(basis_x[1] * basis_x[1] + basis_y[1] * basis_y[1])
  radial_z = radius * wp.sqrt(basis_x[2] * basis_x[2] + basis_y[2] * basis_y[2])

  extent = wp.vec3(
    radial_x + half_height * axis_abs[0],
    radial_y + half_height * axis_abs[1],
    radial_z + half_height * axis_abs[2],
  )

  return pos - extent, pos + extent


@wp.kernel
def _compute_bvh_bounds(
  # Model:
  geom_type: wp.array(dtype=int),
  geom_dataid: wp.array(dtype=int),
  geom_size: wp.array2d(dtype=wp.vec3),
  # Data in:
  geom_xpos_in: wp.array2d(dtype=wp.vec3),
  geom_xmat_in: wp.array2d(dtype=wp.mat33),
  nworld_in: int,
  # In:
  bvh_ngeom: int,
  enabled_geom_ids: wp.array(dtype=int),
  mesh_bounds_size: wp.array(dtype=wp.vec3),
  hfield_bounds_size: wp.array(dtype=wp.vec3),
  # Out:
  lower_out: wp.array(dtype=wp.vec3),
  upper_out: wp.array(dtype=wp.vec3),
  group_out: wp.array(dtype=int),
):
  world_id, geom_local_id = wp.tid()
  geom_id = enabled_geom_ids[geom_local_id]

  pos = geom_xpos_in[world_id, geom_id]
  rot = geom_xmat_in[world_id, geom_id]
  size = geom_size[world_id % geom_size.shape[0], geom_id]
  type = geom_type[geom_id]

  # TODO: Investigate branch elimination with static loop unrolling
  if type == GeomType.SPHERE:
    lower_bound, upper_bound = _compute_sphere_bounds(pos, rot, size)
  elif type == GeomType.CAPSULE:
    lower_bound, upper_bound = _compute_capsule_bounds(pos, rot, size)
  elif type == GeomType.PLANE:
    lower_bound, upper_bound = _compute_plane_bounds(pos, rot, size)
  elif type == GeomType.MESH:
    size = mesh_bounds_size[geom_dataid[geom_id]]
    lower_bound, upper_bound = _compute_box_bounds(pos, rot, size)
  elif type == GeomType.ELLIPSOID:
    lower_bound, upper_bound = _compute_ellipsoid_bounds(pos, rot, size)
  elif type == GeomType.CYLINDER:
    lower_bound, upper_bound = _compute_cylinder_bounds(pos, rot, size)
  elif type == GeomType.BOX:
    lower_bound, upper_bound = _compute_box_bounds(pos, rot, size)
  elif type == GeomType.HFIELD:
    size = hfield_bounds_size[geom_dataid[geom_id]]
    lower_bound, upper_bound = _compute_box_bounds(pos, rot, size)

  lower_out[world_id * bvh_ngeom + geom_local_id] = lower_bound
  upper_out[world_id * bvh_ngeom + geom_local_id] = upper_bound
  group_out[world_id * bvh_ngeom + geom_local_id] = world_id


@wp.kernel
def compute_bvh_group_roots(
  # In:
  bvh_id: wp.uint64,
  # Out:
  group_root_out: wp.array(dtype=int),
):
  tid = wp.tid()
  root = wp.bvh_get_group_root(bvh_id, tid)
  group_root_out[tid] = root


def build_scene_bvh(m: Model, d: Data, rc: RenderContext):
  """Build a global BVH for all geometries in all worlds."""
  wp.launch(
    kernel=_compute_bvh_bounds,
    dim=(d.nworld, rc.bvh_ngeom),
    inputs=[
      m.geom_type,
      m.geom_dataid,
      m.geom_size,
      d.geom_xpos,
      d.geom_xmat,
      d.nworld,
      rc.bvh_ngeom,
      rc.enabled_geom_ids,
      rc.mesh_bounds_size,
      rc.hfield_bounds_size,
      rc.lower,
      rc.upper,
      rc.group,
    ],
  )

  bvh = wp.Bvh(rc.lower, rc.upper, groups=rc.group, constructor="sah")

  # BVH handle must be stored to avoid garbage collection
  rc.bvh = bvh
  rc.bvh_id = bvh.id

  wp.launch(
    kernel=compute_bvh_group_roots,
    dim=d.nworld,
    inputs=[bvh.id],
    outputs=[rc.group_root],
  )


def refit_scene_bvh(m: Model, d: Data, rc: RenderContext):
  wp.launch(
    kernel=_compute_bvh_bounds,
    dim=(d.nworld, rc.bvh_ngeom),
    inputs=[
      m.geom_type,
      m.geom_dataid,
      m.geom_size,
      d.geom_xpos,
      d.geom_xmat,
      d.nworld,
      rc.bvh_ngeom,
      rc.enabled_geom_ids,
      rc.mesh_bounds_size,
      rc.hfield_bounds_size,
      rc.lower,
      rc.upper,
      rc.group,
    ],
  )

  rc.bvh.refit()


def build_mesh_bvh(
  mjm: mujoco.MjModel,
  meshid: int,
  constructor: str = "sah",
  leaf_size: int = 2,
) -> tuple[wp.Mesh, wp.vec3]:
  """Create a Warp mesh BVH from mesh data."""
  v_start = mjm.mesh_vertadr[meshid]
  v_end = v_start + mjm.mesh_vertnum[meshid]
  points = mjm.mesh_vert[v_start:v_end]

  f_start = mjm.mesh_faceadr[meshid]
  f_end = mjm.mesh_face.shape[0] if (meshid + 1) >= mjm.mesh_faceadr.shape[0] else mjm.mesh_faceadr[meshid + 1]
  indices = mjm.mesh_face[f_start:f_end]
  indices = indices.flatten()
  pmin = np.min(points, axis=0)
  pmax = np.max(points, axis=0)
  half = 0.5 * (pmax - pmin)

  points = wp.array(points, dtype=wp.vec3)
  indices = wp.array(indices, dtype=wp.int32)
  mesh = wp.Mesh(points=points, indices=indices, bvh_constructor=constructor, bvh_leaf_size=leaf_size)

  return mesh, half


def _optimize_hfield_mesh(
  data: np.ndarray,
  nr: int,
  nc: int,
  sx: float,
  sy: float,
  sz_scale: float,
  width: float,
  height: float,
) -> tuple[np.ndarray, np.ndarray]:
  """Greedy meshing for heightfield optimization.

  Merges coplanar adjacent cells into larger rectangles to
  reduce triangle and vertex count.
  """
  points_map = {}
  points_list = []
  indices_list = []

  def get_point_index(r, c):
    if (r, c) in points_map:
      return points_map[(r, c)]

    # Compute vertex position
    x = sx * (float(c) / width - 1.0)
    y = sy * (float(r) / height - 1.0)
    z = float(data[r, c]) * sz_scale

    idx = len(points_list)
    points_list.append([x, y, z])
    points_map[(r, c)] = idx
    return idx

  visited = np.zeros((nr - 1, nc - 1), dtype=bool)

  for r in range(nr - 1):
    for c in range(nc - 1):
      if visited[r, c]:
        continue

      # Check if current cell is planar
      z00 = data[r, c]
      z01 = data[r, c + 1]
      z10 = data[r + 1, c]
      z11 = data[r + 1, c + 1]

      # Approx check for planarity: z00 + z11 == z01 + z10
      is_planar = abs((z00 + z11) - (z01 + z10)) < 1e-5

      if not is_planar:
        # Must emit single cell (2 triangles)
        idx00 = get_point_index(r, c)
        idx01 = get_point_index(r, c + 1)
        idx10 = get_point_index(r + 1, c)
        idx11 = get_point_index(r + 1, c + 1)

        # Tri 1: TL, TR, BR
        indices_list.extend([idx00, idx01, idx11])
        # Tri 2: TL, BR, BL
        indices_list.extend([idx00, idx11, idx10])
        visited[r, c] = True
        continue

      # If planar, try to expand
      slope_x = z01 - z00
      slope_y = z10 - z00
      w = 1
      h = 1

      def fits_plane(rr, cc):
        if rr >= nr - 1 or cc >= nc - 1:
          return False
        # Check planarity of the cell itself
        cz00 = data[rr, cc]
        cz01 = data[rr, cc + 1]
        cz10 = data[rr + 1, cc]
        cz11 = data[rr + 1, cc + 1]
        if abs((cz00 + cz11) - (cz01 + cz10)) >= 1e-5:
          return False

        # Check if it lies on the SAME plane as start cell
        # Expected z at (rr, cc)
        z_pred = z00 + (rr - r) * slope_y + (cc - c) * slope_x
        if abs(cz00 - z_pred) >= 1e-5:
          return False

        # Since cell is planar and one corner matches, slopes must match if connected
        cslope_x = cz01 - cz00
        cslope_y = cz10 - cz00
        if abs(cslope_x - slope_x) >= 1e-5 or abs(cslope_y - slope_y) >= 1e-5:
          return False

        return True

      # Expand width
      while c + w < nc - 1 and not visited[r, c + w] and fits_plane(r, c + w):
        w += 1

      # Expand height
      while r + h < nr - 1:
        # Check entire row
        row_ok = True
        for k in range(w):
          if visited[r + h, c + k] or not fits_plane(r + h, c + k):
            row_ok = False
            break
        if row_ok:
          h += 1
        else:
          break

      # Mark visited
      visited[r : r + h, c : c + w] = True

      # Emit large quad
      idx_tl = get_point_index(r, c)
      idx_tr = get_point_index(r, c + w)
      idx_bl = get_point_index(r + h, c)
      idx_br = get_point_index(r + h, c + w)

      # Tri 1: TL, TR, BR
      indices_list.extend([idx_tl, idx_tr, idx_br])
      # Tri 2: TL, BR, BL
      indices_list.extend([idx_tl, idx_br, idx_bl])

  return np.array(points_list, dtype=np.float32), np.array(indices_list, dtype=np.int32)


def build_hfield_bvh(
  mjm: mujoco.MjModel,
  hfieldid: int,
  constructor: str = "sah",
  leaf_size: int = 2,
) -> tuple[wp.Mesh, wp.vec3]:
  """Create a Warp mesh BVH from heightfield data."""
  nr = mjm.hfield_nrow[hfieldid]
  nc = mjm.hfield_ncol[hfieldid]
  sz = np.asarray(mjm.hfield_size[hfieldid], dtype=np.float32)

  adr = mjm.hfield_adr[hfieldid]
  data = mjm.hfield_data[adr : adr + nr * nc].reshape((nr, nc))

  width = 0.5 * max(nc - 1, 1)
  height = 0.5 * max(nr - 1, 1)

  points, indices = _optimize_hfield_mesh(
    data,
    nr,
    nc,
    sz[0],
    sz[1],
    sz[2],
    width,
    height,
  )
  pmin = np.min(points, axis=0)
  pmax = np.max(points, axis=0)
  half = 0.5 * (pmax - pmin)

  points = wp.array(points, dtype=wp.vec3)
  indices = wp.array(indices, dtype=wp.int32)

  mesh = wp.Mesh(
    points=points,
    indices=indices,
    bvh_constructor=constructor,
    bvh_leaf_size=leaf_size,
  )

  return mesh, half


@wp.kernel
def accumulate_flex_vertex_normals(
  # Model:
  flex_elem: wp.array(dtype=int),
  # Data in:
  flexvert_xpos_in: wp.array2d(dtype=wp.vec3),
  # Out:
  flexvert_norm_out: wp.array2d(dtype=wp.vec3),
):
  """Accumulate per-vertex normals by summing adjacent face normals."""
  worldid, elemid = wp.tid()

  elem_base = elemid * 3
  i0 = flex_elem[elem_base + 0]
  i1 = flex_elem[elem_base + 1]
  i2 = flex_elem[elem_base + 2]

  v0 = flexvert_xpos_in[worldid, i0]
  v1 = flexvert_xpos_in[worldid, i1]
  v2 = flexvert_xpos_in[worldid, i2]

  face_nrm = wp.cross(v1 - v0, v2 - v0)
  face_nrm = wp.normalize(face_nrm)
  flexvert_norm_out[worldid, i0] += face_nrm
  flexvert_norm_out[worldid, i1] += face_nrm
  flexvert_norm_out[worldid, i2] += face_nrm


@wp.kernel
def normalize_vertex_normals(
  # Out:
  flexvert_norm_out: wp.array2d(dtype=wp.vec3),
):
  """Normalize accumulated vertex normals."""
  worldid, vertid = wp.tid()
  flexvert_norm_out[worldid, vertid] = wp.normalize(flexvert_norm_out[worldid, vertid])


@wp.kernel
def _build_flex_2d_elements(
  # Model:
  flex_elem: wp.array(dtype=int),
  # Data in:
  flexvert_xpos_in: wp.array2d(dtype=wp.vec3),
  # In:
  flexvert_norm_in: wp.array2d(dtype=wp.vec3),
  elem_adr: int,
  vert_adr: int,
  face_offset: int,
  radius: float,
  nfaces: int,
  # Out:
  face_point_out: wp.array(dtype=wp.vec3),
  face_index_out: wp.array(dtype=int),
  group_out: wp.array(dtype=int),
):
  """Create faces from 2D flex elements (triangles).

  Two faces (top/bottom) per element, separated by the radius of the flex element.
  """
  worldid, elemid = wp.tid()

  base = elem_adr + elemid * 3
  i0 = vert_adr + flex_elem[base + 0]
  i1 = vert_adr + flex_elem[base + 1]
  i2 = vert_adr + flex_elem[base + 2]

  v0 = flexvert_xpos_in[worldid, i0]
  v1 = flexvert_xpos_in[worldid, i1]
  v2 = flexvert_xpos_in[worldid, i2]

  n0 = flexvert_norm_in[worldid, i0]
  n1 = flexvert_norm_in[worldid, i1]
  n2 = flexvert_norm_in[worldid, i2]

  p0_pos = v0 + radius * n0
  p1_pos = v1 + radius * n1
  p2_pos = v2 + radius * n2

  p0_neg = v0 - radius * n0
  p1_neg = v1 - radius * n1
  p2_neg = v2 - radius * n2

  world_face_offset = worldid * nfaces

  # First face (top): i0, i1, i2
  face_id0 = world_face_offset + face_offset + 2 * elemid
  base0 = face_id0 * 3
  face_point_out[base0 + 0] = p0_pos
  face_point_out[base0 + 1] = p1_pos
  face_point_out[base0 + 2] = p2_pos

  face_index_out[base0 + 0] = base0 + 0
  face_index_out[base0 + 1] = base0 + 1
  face_index_out[base0 + 2] = base0 + 2

  group_out[face_id0] = worldid

  # Second face (bottom): i0, i2, i1 (opposite winding)
  face_id1 = world_face_offset + face_offset + 2 * elemid + 1
  base1 = face_id1 * 3
  face_point_out[base1 + 0] = p0_neg
  face_point_out[base1 + 1] = p1_neg
  face_point_out[base1 + 2] = p2_neg

  face_index_out[base1 + 0] = base1 + 0
  face_index_out[base1 + 1] = base1 + 2
  face_index_out[base1 + 2] = base1 + 1

  group_out[face_id1] = worldid


@wp.kernel
def _build_flex_2d_sides(
  # Data in:
  flexvert_xpos_in: wp.array2d(dtype=wp.vec3),
  # In:
  flexvert_norm_in: wp.array2d(dtype=wp.vec3),
  flex_shell_in: wp.array(dtype=int),
  shell_adr: int,
  vert_adr: int,
  face_offset: int,
  radius: float,
  nface: int,
  # Out:
  face_point_out: wp.array(dtype=wp.vec3),
  face_index_out: wp.array(dtype=int),
  group_out: wp.array(dtype=int),
):
  """Create side faces from 2D flex shell fragments.

  For each shell fragment (edge i0 -> i1), we emit two triangles:
    - one using +radius
    - one using -radius (i0/i1 swapped)
  """
  worldid, shellid = wp.tid()

  base = shell_adr + 2 * shellid
  i0 = vert_adr + flex_shell_in[base + 0]
  i1 = vert_adr + flex_shell_in[base + 1]

  v0 = flexvert_xpos_in[worldid, i0]
  v1 = flexvert_xpos_in[worldid, i1]

  n0 = flexvert_norm_in[worldid, i0]
  n1 = flexvert_norm_in[worldid, i1]

  neg_radius = -radius

  # First side i0, i1 with +radius
  face_id0 = worldid * nface + face_offset + 2 * shellid
  base0 = face_id0 * 3
  face_point_out[base0 + 0] = v0 + n0 * radius
  face_point_out[base0 + 1] = v1 + n1 * neg_radius
  face_point_out[base0 + 2] = v1 + n1 * radius
  face_index_out[base0 + 0] = base0 + 0
  face_index_out[base0 + 1] = base0 + 1
  face_index_out[base0 + 2] = base0 + 2

  # Second side i1, i0 with -radius
  face_id1 = worldid * nface + face_offset + 2 * shellid + 1
  base1 = face_id1 * 3
  face_point_out[base1 + 0] = v1 + n1 * neg_radius
  face_point_out[base1 + 1] = v0 + n0 * neg_radius
  face_point_out[base1 + 2] = v0 + n0 * radius
  face_index_out[base1 + 0] = base1 + 0
  face_index_out[base1 + 1] = base1 + 1
  face_index_out[base1 + 2] = base1 + 2

  group_out[face_id0] = worldid
  group_out[face_id1] = worldid


@wp.kernel
def _build_flex_3d_shells(
  # Data in:
  flexvert_xpos_in: wp.array2d(dtype=wp.vec3),
  # In:
  flex_shell_in: wp.array(dtype=int),
  shell_adr: int,
  vert_adr: int,
  face_offset: int,
  nface: int,
  # Out:
  face_point_out: wp.array(dtype=wp.vec3),
  face_index_out: wp.array(dtype=int),
  group_out: wp.array(dtype=int),
):
  """Create faces from 3D flex shell fragments (triangles).

  Each shell fragment contributes a single triangle whose vertices are taken
  directly from the flex vertex positions (one-sided surface).
  """
  worldid, shellid = wp.tid()

  base = shell_adr + shellid * 3
  i0 = vert_adr + flex_shell_in[base + 0]
  i1 = vert_adr + flex_shell_in[base + 1]
  i2 = vert_adr + flex_shell_in[base + 2]

  face_id = worldid * nface + face_offset + shellid
  base = face_id * 3

  v0 = flexvert_xpos_in[worldid, i0]
  v1 = flexvert_xpos_in[worldid, i1]
  v2 = flexvert_xpos_in[worldid, i2]

  face_point_out[base + 0] = v0
  face_point_out[base + 1] = v1
  face_point_out[base + 2] = v2

  face_index_out[base + 0] = base + 0
  face_index_out[base + 1] = base + 1
  face_index_out[base + 2] = base + 2

  group_out[face_id] = worldid


@wp.kernel
def _update_flex_face_points(
  # Model:
  nflex: int,
  flex_dim: wp.array(dtype=int),
  flex_vertadr: wp.array(dtype=int),
  flex_elemnum: wp.array(dtype=int),
  flex_elem: wp.array(dtype=int),
  # Data in:
  flexvert_xpos_in: wp.array2d(dtype=wp.vec3),
  # In:
  flex_shell_in: wp.array(dtype=int),
  flexvert_norm_in: wp.array2d(dtype=wp.vec3),
  flex_elemdataadr: wp.array(dtype=int),
  flex_shelldataadr: wp.array(dtype=int),
  flex_faceadr: wp.array(dtype=int),
  flex_radius: wp.array(dtype=float),
  flex_workadr: wp.array(dtype=int),
  flex_worknum: wp.array(dtype=int),
  nfaces: int,
  smooth: bool,
  # Out:
  face_point_out: wp.array(dtype=wp.vec3),
):
  worldid, workid = wp.tid()

  # identify which flex this work item belongs to
  f = int(0)
  locid = int(0)
  for i in range(nflex):
    locid = workid - flex_workadr[i]
    if locid >= 0 and locid < flex_worknum[i]:
      f = i
      break

  dim = flex_dim[f]
  face_offset = flex_faceadr[f]
  world_face_offset = worldid * nfaces
  vert_adr = flex_vertadr[f]

  if dim == 2:
    radius = flex_radius[f]
    elem_count = flex_elemnum[f]

    if locid < elem_count:
      # 2D element faces
      elemid = locid
      elem_adr = flex_elemdataadr[f]
      ebase = elem_adr + elemid * 3
      i0 = vert_adr + flex_elem[ebase + 0]
      i1 = vert_adr + flex_elem[ebase + 1]
      i2 = vert_adr + flex_elem[ebase + 2]

      v0 = flexvert_xpos_in[worldid, i0]
      v1 = flexvert_xpos_in[worldid, i1]
      v2 = flexvert_xpos_in[worldid, i2]

      # TODO: Use static conditional
      if smooth:
        n0 = flexvert_norm_in[worldid, i0]
        n1 = flexvert_norm_in[worldid, i1]
        n2 = flexvert_norm_in[worldid, i2]
      else:
        face_nrm = wp.cross(v1 - v0, v2 - v0)
        face_nrm = wp.normalize(face_nrm)
        n0 = face_nrm
        n1 = face_nrm
        n2 = face_nrm

      p0_pos = v0 + radius * n0
      p1_pos = v1 + radius * n1
      p2_pos = v2 + radius * n2

      p0_neg = v0 - radius * n0
      p1_neg = v1 - radius * n1
      p2_neg = v2 - radius * n2

      face_id0 = world_face_offset + face_offset + (2 * elemid)
      base0 = face_id0 * 3
      face_point_out[base0 + 0] = p0_pos
      face_point_out[base0 + 1] = p1_pos
      face_point_out[base0 + 2] = p2_pos

      face_id1 = world_face_offset + face_offset + (2 * elemid + 1)
      base1 = face_id1 * 3
      face_point_out[base1 + 0] = p0_neg
      face_point_out[base1 + 1] = p1_neg
      face_point_out[base1 + 2] = p2_neg
    else:
      # 2D shell faces
      shellid = locid - elem_count
      shell_adr = flex_shelldataadr[f]
      sbase = shell_adr + 2 * shellid
      i0 = vert_adr + flex_shell_in[sbase + 0]
      i1 = vert_adr + flex_shell_in[sbase + 1]

      v0 = flexvert_xpos_in[worldid, i0]
      v1 = flexvert_xpos_in[worldid, i1]

      n0 = flexvert_norm_in[worldid, i0]
      n1 = flexvert_norm_in[worldid, i1]

      shell_face_offset = face_offset + (2 * elem_count)
      face_id0 = world_face_offset + shell_face_offset + (2 * shellid)
      base0 = face_id0 * 3
      face_point_out[base0 + 0] = v0 + radius * n0
      face_point_out[base0 + 1] = v1 - radius * n1
      face_point_out[base0 + 2] = v1 + radius * n1

      face_id1 = world_face_offset + shell_face_offset + (2 * shellid + 1)
      base1 = face_id1 * 3
      face_point_out[base1 + 0] = v1 - radius * n1
      face_point_out[base1 + 1] = v0 + radius * n0
      face_point_out[base1 + 2] = v0 - radius * n0
  else:
    # 3D shell faces
    shellid = locid
    shell_adr = flex_shelldataadr[f]
    sbase = shell_adr + shellid * 3
    i0 = vert_adr + flex_shell_in[sbase + 0]
    i1 = vert_adr + flex_shell_in[sbase + 1]
    i2 = vert_adr + flex_shell_in[sbase + 2]

    v0 = flexvert_xpos_in[worldid, i0]
    v1 = flexvert_xpos_in[worldid, i1]
    v2 = flexvert_xpos_in[worldid, i2]

    face_id = world_face_offset + face_offset + shellid
    fbase = face_id * 3

    face_point_out[fbase + 0] = v0
    face_point_out[fbase + 1] = v1
    face_point_out[fbase + 2] = v2


def build_flex_bvh(
  mjm: mujoco.MjModel, m: Model, d: Data, constructor: str = "sah", leaf_size: int = 2
) -> tuple[wp.Mesh, wp.array, wp.array, wp.array, wp.array, wp.array, int]:
  """Create a Warp mesh BVH from flex data."""
  if (mjm.flex_dim == 1).any():
    raise ValueError("1D Flex objects are not currently supported.")

  nflex = mjm.nflex

  flex_faceadr = [0]
  for f in range(nflex):
    if mjm.flex_dim[f] == 2:
      flex_faceadr.append(flex_faceadr[-1] + 2 * mjm.flex_elemnum[f] + 2 * mjm.flex_shellnum[f])
    elif mjm.flex_dim[f] == 3:
      flex_faceadr.append(flex_faceadr[-1] + mjm.flex_shellnum[f])

  nface = int(flex_faceadr[-1])
  flex_faceadr = flex_faceadr[:-1]

  face_point = wp.zeros(nface * 3 * d.nworld, dtype=wp.vec3)
  face_index = wp.zeros(nface * 3 * d.nworld, dtype=wp.int32)
  group = wp.zeros(nface * d.nworld, dtype=int)

  flexvert_norm = wp.zeros(d.flexvert_xpos.shape, dtype=wp.vec3)
  flex_shell = wp.array(mjm.flex_shell, dtype=int)

  wp.launch(
    kernel=accumulate_flex_vertex_normals,
    dim=(d.nworld, m.nflexelemdata // 3),
    inputs=[m.flex_elem, d.flexvert_xpos],
    outputs=[flexvert_norm],
  )

  wp.launch(
    kernel=normalize_vertex_normals,
    dim=(d.nworld, m.nflexvert),
    inputs=[flexvert_norm],
  )

  for f in range(nflex):
    dim = mjm.flex_dim[f]
    elem_adr = mjm.flex_elemdataadr[f]
    nelem = mjm.flex_elemnum[f]
    shell_adr = mjm.flex_shelldataadr[f]
    nshell = mjm.flex_shellnum[f]
    vert_adr = mjm.flex_vertadr[f]

    if dim == 2:
      wp.launch(
        kernel=_build_flex_2d_elements,
        dim=(d.nworld, nelem),
        inputs=[
          m.flex_elem,
          d.flexvert_xpos,
          flexvert_norm,
          elem_adr,
          vert_adr,
          flex_faceadr[f],
          mjm.flex_radius[f],
          nface,
        ],
        outputs=[face_point, face_index, group],
      )

      wp.launch(
        kernel=_build_flex_2d_sides,
        dim=(d.nworld, nshell),
        inputs=[
          d.flexvert_xpos,
          flexvert_norm,
          flex_shell,
          shell_adr,
          vert_adr,
          flex_faceadr[f] + 2 * nelem,
          mjm.flex_radius[f],
          nface,
        ],
        outputs=[face_point, face_index, group],
      )
    elif dim == 3:
      wp.launch(
        kernel=_build_flex_3d_shells,
        dim=(d.nworld, nshell),
        inputs=[
          d.flexvert_xpos,
          flex_shell,
          shell_adr,
          vert_adr,
          flex_faceadr[f],
          nface,
        ],
        outputs=[face_point, face_index, group],
      )

  flex_mesh = wp.Mesh(
    points=face_point,
    indices=face_index,
    groups=group,
    bvh_constructor=constructor,
    bvh_leaf_size=leaf_size,
  )

  group_root = wp.zeros(d.nworld, dtype=int)
  wp.launch(
    kernel=compute_bvh_group_roots,
    dim=d.nworld,
    inputs=[flex_mesh.id],
    outputs=[group_root],
  )

  return (
    flex_mesh,
    face_point,
    group_root,
    flex_shell,
    flex_faceadr,
    nface,
  )


def refit_flex_bvh(m: Model, d: Data, rc: RenderContext):
  """Refit the flex BVH."""
  flexvert_norm = wp.zeros(d.flexvert_xpos.shape, dtype=wp.vec3)

  wp.launch(
    kernel=accumulate_flex_vertex_normals,
    dim=(d.nworld, m.nflexelemdata // 3),
    inputs=[
      m.flex_elem,
      d.flexvert_xpos,
    ],
    outputs=[flexvert_norm],
  )

  wp.launch(
    kernel=normalize_vertex_normals,
    dim=(d.nworld, m.nflexvert),
    inputs=[flexvert_norm],
  )

  wp.launch(
    kernel=_update_flex_face_points,
    dim=(d.nworld, rc.flex_nwork),
    inputs=[
      m.nflex,
      m.flex_dim,
      m.flex_vertadr,
      m.flex_elemnum,
      m.flex_elem,
      d.flexvert_xpos,
      rc.flex_shell,
      flexvert_norm,
      rc.flex_elemdataadr,
      rc.flex_shelldataadr,
      rc.flex_faceadr,
      rc.flex_radius,
      rc.flex_workadr,
      rc.flex_worknum,
      rc.flex_nface,
      rc.flex_render_smooth,
    ],
    outputs=[rc.flex_face_point],
  )

  rc.flex_mesh.refit()
