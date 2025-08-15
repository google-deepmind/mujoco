# Copyright 2023 DeepMind Technologies Limited
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
"""Functions for ray interesection testing."""

from functools import partial
from typing import Sequence, Tuple

import jax
from jax import numpy as jp
from jax.scipy.ndimage import map_coordinates
import mujoco
from mujoco.mjx._src import math
# pylint: disable=g-importing-member
from mujoco.mjx._src.types import Data
from mujoco.mjx._src.types import GeomType
from mujoco.mjx._src.types import Model
# pylint: enable=g-importing-member
import numpy as np


def _ray_quad(
    a: jax.Array, b: jax.Array, c: jax.Array
) -> Tuple[jax.Array, jax.Array]:
  """Returns two solutions for quadratic: a*x^2 + 2*b*x + c = 0."""
  det = b * b - a * c
  det_2 = jp.sqrt(det)

  x0, x1 = math.safe_div(-b - det_2, a), math.safe_div(-b + det_2, a)
  x0 = jp.where((det < mujoco.mjMINVAL) | (x0 < 0), jp.inf, x0)
  x1 = jp.where((det < mujoco.mjMINVAL) | (x1 < 0), jp.inf, x1)

  return x0, x1


def _ray_plane(
    size: jax.Array,
    pnt: jax.Array,
    vec: jax.Array,
) -> jax.Array:
  """Returns the distance at which a ray intersects with a plane."""
  x = -math.safe_div(pnt[2], vec[2])

  valid = vec[2] <= -mujoco.mjMINVAL  # z-vec pointing towards front face
  valid &= x >= 0
  # only within rendered rectangle
  p = pnt[0:2] + x * vec[0:2]
  valid &= jp.all((size[0:2] <= 0) | (jp.abs(p) <= size[0:2]))

  return jp.where(valid, x, jp.inf)


def _ray_sphere(
    size: jax.Array,
    pnt: jax.Array,
    vec: jax.Array,
) -> jax.Array:
  """Returns the distance at which a ray intersects with a sphere."""
  x0, x1 = _ray_quad(vec @ vec, vec @ pnt, pnt @ pnt - size[0] * size[0])
  x = jp.where(jp.isinf(x0), x1, x0)

  return x


def _ray_capsule(
    size: jax.Array,
    pnt: jax.Array,
    vec: jax.Array,
) -> jax.Array:
  """Returns the distance at which a ray intersects with a capsule."""

  # cylinder round side: (x*lvec+lpnt)'*(x*lvec+lpnt) = size[0]*size[0]
  a = vec[0:2] @ vec[0:2]
  b = vec[0:2] @ pnt[0:2]
  c = pnt[0:2] @ pnt[0:2] - size[0] * size[0]

  # solve a*x^2 + 2*b*x + c = 0
  x0, x1 = _ray_quad(a, b, c)
  x = jp.where(jp.isinf(x0), x1, x0)

  # make sure round solution is between flat sides
  x = jp.where(jp.abs(pnt[2] + x * vec[2]) <= size[1], x, jp.inf)

  # top cap
  dif = pnt - jp.array([0, 0, size[1]])
  x0, x1 = _ray_quad(vec @ vec, vec @ dif, dif @ dif - size[0] * size[0])
  # accept only top half of sphere
  x = jp.where((pnt[2] + x0 * vec[2] >= size[1]) & (x0 < x), x0, x)
  x = jp.where((pnt[2] + x1 * vec[2] >= size[1]) & (x1 < x), x1, x)

  # bottom cap
  dif = pnt + jp.array([0, 0, size[1]])
  x0, x1 = _ray_quad(vec @ vec, vec @ dif, dif @ dif - size[0] * size[0])

  # accept only bottom half of sphere
  x = jp.where((pnt[2] + x0 * vec[2] <= -size[1]) & (x0 < x), x0, x)
  x = jp.where((pnt[2] + x1 * vec[2] <= -size[1]) & (x1 < x), x1, x)

  return x


def _ray_ellipsoid(
    size: jax.Array,
    pnt: jax.Array,
    vec: jax.Array,
) -> jax.Array:
  """Returns the distance at which a ray intersects with an ellipsoid."""

  # invert size^2
  s = math.safe_div(1, jp.square(size))

  # (x*lvec+lpnt)' * diag(1/size^2) * (x*lvec+lpnt) = 1
  svec = s * vec
  a = svec @ vec
  b = svec @ pnt
  c = (s * pnt) @ pnt - 1

  # solve a*x^2 + 2*b*x + c = 0
  x0, x1 = _ray_quad(a, b, c)
  x = jp.where(jp.isinf(x0), x1, x0)

  return x


def _ray_box(
    size: jax.Array,
    pnt: jax.Array,
    vec: jax.Array,
) -> jax.Array:
  """Returns the distance at which a ray intersects with a box."""

  iface = jp.array([(1, 2), (0, 2), (0, 1), (1, 2), (0, 2), (0, 1)])

  # side +1, -1
  # solution of pnt[i] + x * vec[i] = side * size[i]
  x = jp.concatenate([math.safe_div(size - pnt,  vec), -math.safe_div(size + pnt, vec)])

  # intersection with face
  p0 = pnt[iface[:, 0]] + x * vec[iface[:, 0]]
  p1 = pnt[iface[:, 1]] + x * vec[iface[:, 1]]
  valid = jp.abs(p0) <= size[iface[:, 0]]
  valid &= jp.abs(p1) <= size[iface[:, 1]]
  valid &= x >= 0

  return jp.min(jp.where(valid, x, jp.inf))


def _ray_box_6(
    size: jax.Array, pnt: jax.Array, vec: jax.Array
) -> jax.Array:
  """Returns intersection distances for all 6 faces of a box."""
  # replace zero vec components with small number to avoid division by zero
  safe_vec = jp.where(jp.abs(vec) < mujoco.mjMINVAL, mujoco.mjMINVAL, vec)
  iface = jp.array([(1, 2), (1, 2), (0, 2), (0, 2), (0, 1), (0, 1)])

  # distances to planes for each of the 6 faces (+x, -x, +y, -y, +z, -z)
  x = jp.concatenate([(size - pnt) / safe_vec, (-size - pnt) / safe_vec])

  # check if intersection points are within face bounds
  p_intersect = pnt + x[:, None] * vec
  p_check_dim1 = jp.abs(p_intersect[jp.arange(6), iface[:, 0]])
  p_check_dim2 = jp.abs(p_intersect[jp.arange(6), iface[:, 1]])
  valid = (p_check_dim1 <= size[iface[:, 0]]) & (
      p_check_dim2 <= size[iface[:, 1]]
  )
  valid &= x >= 0

  return jp.where(valid, x, jp.inf)


def _ray_triangle(
    vert: jax.Array,
    pnt: jax.Array,
    vec: jax.Array,
    basis: jax.Array,
) -> jax.Array:
  """Returns the distance at which a ray intersects with a triangle."""
  # project difference vectors in ray normal plane
  planar = jp.dot(vert - pnt, basis)

  # determine if origin is inside planar projection of triangle
  # A = (p0-p2, p1-p2), b = -p2, solve A*t = b
  A = planar[0:2] - planar[2]  # pylint: disable=invalid-name
  b = -planar[2]
  det = A[0, 0] * A[1, 1] - A[1, 0] * A[0, 1]

  t0 = math.safe_div(A[1, 1] * b[0] - A[1, 0] * b[1], det)
  t1 = math.safe_div(-A[0, 1] * b[0] + A[0, 0] * b[1], det)
  valid = (t0 >= 0) & (t1 >= 0) & (t0 + t1 <= 1)

  # intersect ray with plane of triangle
  nrm = jp.cross(vert[0] - vert[2], vert[1] - vert[2])
  dist = math.safe_div(jp.dot(vert[2] - pnt, nrm), jp.dot(vec, nrm))
  valid &= dist >= 0
  dist = jp.where(valid, dist, jp.inf)

  return dist


def _ray_mesh(
    m: Model,
    geom_id: np.ndarray,
    unused_size: jax.Array,
    pnt: jax.Array,
    vec: jax.Array,
) -> Tuple[jax.Array, jax.Array]:
  """Returns the best distance and geom_id for ray mesh intersections."""
  data_id = m.geom_dataid[geom_id]

  ray_basis = lambda x: jp.array(math.orthogonals(math.normalize(x))).T
  basis = jax.vmap(ray_basis)(vec)

  faceadr = np.append(m.mesh_faceadr, m.nmeshface)
  vertadr = np.append(m.mesh_vertadr, m.nmeshvert)

  dists, geom_ids = [], []
  for i, id_ in enumerate(data_id):
    face = m.mesh_face[faceadr[id_] : faceadr[id_ + 1]]
    vert = m.mesh_vert[vertadr[id_] : vertadr[id_ + 1]]
    vert = jp.array(vert[face])
    dist = jax.vmap(_ray_triangle, in_axes=(0, None, None, None))(
        vert, pnt[i], vec[i], basis[i]
    )
    dists.append(dist)
    geom_ids.append(np.repeat(geom_id[i], dist.size))

  dists = jp.concatenate(dists)
  min_id = jp.argmin(dists)
  # Grab the best distance amongst all meshes, bypassing the argmin in `ray`.
  # This avoids having to compute the best distance per mesh.
  dist = dists[min_id, None]
  id_ = jp.array(np.concatenate(geom_ids))[min_id, None]

  return dist, id_

@partial(jax.jit, static_argnames=('nrow', 'ncol'))
def _ray_hfield_static(
    hfield_data_flat: jax.Array,
    size: jax.Array,
    pnt: jax.Array,
    vec: jax.Array,
    adr: jax.Array,
    nrow: int,
    ncol: int,
) -> jax.Array:
  """JIT-compiled kernel to raycast against a single hfield size.
  """
  # size: (xy_size_x, xy_size_y, height_range, base_thickness)
  # Intersection with base box
  base_size = jp.array([size[0], size[1], size[3] / 2.0])
  base_pos = jp.array([0, 0, -size[3] / 2.0])
  dist = _ray_box(base_size, pnt - base_pos, vec)

  # Intersection with top box (containing terrain)
  top_size = jp.array([size[0], size[1], size[2] / 2.0])
  top_pos = jp.array([0, 0, size[2] / 2.0])
  top_dists_all = _ray_box_6(top_size, pnt - top_pos, vec)
  top_dist_min = jp.min(top_dists_all, initial=jp.inf)

  def _intersect_surface() -> jax.Array:
    r_idx_grid, c_idx_grid = jp.meshgrid(
        jp.arange(nrow), jp.arange(ncol), indexing='ij'
    )
    flat_indices = (adr + r_idx_grid * ncol + c_idx_grid).flatten()
    hfield_data = hfield_data_flat[flat_indices].reshape((nrow, ncol))

    # 1. Test against all triangles in the grid.
    # Use initial=jp.inf for safety against empty arrays if nrow/ncol <= 1
    min_tri_dist = jp.inf
    if nrow > 1 and ncol > 1:
      dx = 2.0 * size[0] / (ncol - 1)
      dy = 2.0 * size[1] / (nrow - 1)
      x_coords = c_idx_grid * dx - size[0]
      y_coords = r_idx_grid * dy - size[1]
      z_coords = hfield_data * size[2]
      v00 = jp.stack([x_coords[:-1, :-1], y_coords[:-1, :-1], z_coords[:-1, :-1]],
                     axis=-1)
      v10 = jp.stack([x_coords[1:, :-1], y_coords[1:, :-1], z_coords[1:, :-1]],
                     axis=-1)
      v01 = jp.stack([x_coords[:-1, 1:], y_coords[:-1, 1:], z_coords[:-1, 1:]],
                     axis=-1)
      v11 = jp.stack([x_coords[1:, 1:], y_coords[1:, 1:], z_coords[1:, 1:]],
                     axis=-1)
      tri1_verts = jp.stack([v00, v11, v10], axis=-2).reshape(-1, 3, 3)
      tri2_verts = jp.stack([v00, v11, v01], axis=-2).reshape(-1, 3, 3)
      verts = jp.concatenate([tri1_verts, tri2_verts])
      basis = jp.array(math.orthogonals(math.normalize(vec))).T
      tri_dists = jax.vmap(_ray_triangle, in_axes=(0, None, None, None))(
          verts, pnt, vec, basis
      )
      min_tri_dist = jp.min(tri_dists, initial=jp.inf)

    # 2. Test against the four vertical side faces of the top box.
    # Replicates the C-code's 1D linear interpolation
    # for side hits to ensure identical behavior at the boundaries.
    d_sides = top_dists_all[0:4] # Distances for +x, -x, +y, -y faces
    p_sides = pnt + d_sides[:, None] * vec

    safe_dx = jp.where(ncol > 1, 2.0 * size[0] / (ncol - 1), 1.0)
    safe_dy = jp.where(nrow > 1, 2.0 * size[1] / (nrow - 1), 1.0)

    # Handle sides normal to X-axis (+x, -x faces)
    y_float_x_sides = (p_sides[:2, 1] + size[1]) / safe_dy
    y0_x_sides = jp.clip(jp.floor(y_float_x_sides).astype(int), 0, nrow - 2)

    y0_rounded = jp.round(y0_x_sides).astype(int)
    y1_rounded = jp.round(y0_x_sides + 1).astype(int)
    x_indices = jp.array([ncol - 1, 0]) # Grid indices for +x and -x edges
    z0_x = hfield_data[y0_rounded, x_indices]
    z1_x = hfield_data[y1_rounded, x_indices]
    interp_h_x = z0_x * (y0_x_sides + 1 - y_float_x_sides) + z1_x * (
        y_float_x_sides - y0_x_sides
    )

    # Handle sides normal to Y-axis (+y, -y faces)
    x_float_y_sides = (p_sides[2:, 0] + size[0]) / safe_dx
    x0_y_sides = jp.clip(jp.floor(x_float_y_sides).astype(int), 0, ncol - 2)


    x0_rounded = jp.round(x0_y_sides).astype(int)
    x1_rounded = jp.round(x0_y_sides + 1).astype(int)
    y_indices = jp.array([nrow - 1, 0]) # Grid indices for +y and -y edges
    z0_y = hfield_data[y_indices, x0_rounded]
    z1_y = hfield_data[y_indices, x1_rounded]
    interp_h_y = z0_y * (x0_y_sides + 1 - x_float_y_sides) + z1_y * (
        x_float_y_sides - x0_y_sides
    )

    # Combine interpolated heights, scale to world units, and check validity
    interp_h_norm = jp.concatenate([interp_h_x, interp_h_y])
    interp_h = interp_h_norm * size[2]
    valid_side_hit = p_sides[:, 2] < interp_h
    side_dists = jp.where(valid_side_hit, d_sides, jp.inf)
    min_side_dist = jp.min(side_dists, initial=jp.inf)

    return jp.minimum(min_tri_dist, min_side_dist)

  dist_surface = jax.lax.cond(
      jp.isinf(top_dist_min), lambda: jp.inf, _intersect_surface
  )
  return jp.minimum(dist, dist_surface)

_RAY_FUNC = {
    GeomType.PLANE: _ray_plane,
    GeomType.SPHERE: _ray_sphere,
    GeomType.CAPSULE: _ray_capsule,
    GeomType.ELLIPSOID: _ray_ellipsoid,
    GeomType.BOX: _ray_box,
    GeomType.MESH: _ray_mesh,
    GeomType.HFIELD: _ray_hfield_static,
}


def ray(
    m: Model,
    d: Data,
    pnt: jax.Array,
    vec: jax.Array,
    geomgroup: Sequence[int] = (),
    flg_static: bool = True,
    bodyexclude: int = -1,
) -> Tuple[jax.Array, jax.Array]:
  """Returns the geom id and distance at which a ray intersects with a geom.

  Args:
    m: MJX model
    d: MJX data
    pnt: ray origin point (3,)
    vec: ray direction    (3,)
    geomgroup: group inclusion/exclusion mask, or empty to ignore
    flg_static: if True, allows rays to intersect with static geoms
    bodyexclude: ignore geoms on specified body id

  Returns:
    dist: distance from ray origin to geom surface (or -1.0 for no intersection)
    id: id of intersected geom (or -1 for no intersection)
  """
  dists, ids = [], []
  geom_filter = m.geom_bodyid != bodyexclude
  geom_filter &= flg_static | (m.body_weldid[m.geom_bodyid] != 0)
  if geomgroup:
    geomgroup = np.array(geomgroup, dtype=bool)
    geom_filter &= geomgroup[np.clip(m.geom_group, 0, mujoco.mjNGROUP)]

  # map ray to local geom frames
  geom_pnts = jax.vmap(lambda x, y: x.T @ (pnt - y))(d.geom_xmat, d.geom_xpos)
  geom_vecs = jax.vmap(lambda x: x.T @ vec)(d.geom_xmat)

  geom_filter_dyn = (m.geom_matid != -1) | (m.geom_rgba[:, 3] != 0)
  geom_filter_dyn &= (m.geom_matid == -1) | (m.mat_rgba[m.geom_matid, 3] != 0)
  for geom_type, fn in _RAY_FUNC.items():
    (id_,) = np.nonzero(geom_filter & (m.geom_type == geom_type))
    if id_.size == 0:
      continue
    
    if geom_type == GeomType.MESH:
      args = (m, id_, m.geom_size[id_], geom_pnts[id_], geom_vecs[id_])
      dist, id_ = fn(*args)
    elif geom_type == GeomType.HFIELD:
      hfield_dataid = m.geom_dataid[id_]
      nrow = m.hfield_nrow[hfield_dataid][0]
      ncol = m.hfield_ncol[hfield_dataid][0]
      hfield_data_flat = jp.asarray(m.hfield_data)
      hfield_id_for_geoms = m.geom_dataid[id_]

      args = (
          hfield_data_flat,
          m.hfield_size[hfield_id_for_geoms][0],
          geom_pnts[id_][0],
          geom_vecs[id_][0],
          jp.asarray(m.hfield_adr)[hfield_id_for_geoms][0],
          nrow,
          ncol,
      )
      dist = _ray_hfield_static(*args)
    else:
      # remove model and id from args for primitive functions
      args = (m.geom_size[id_], geom_pnts[id_], geom_vecs[id_])
      dist = jax.vmap(fn)(*args)

    dist = jp.where(geom_filter_dyn[id_], dist, jp.inf)
    dists, ids = dists + [dist], ids + [id_]

  if not ids:
    return jp.array(-1.0), jp.array(-1)

  dists = jp.concatenate(dists)
  ids = jp.concatenate(ids)
  min_id = jp.argmin(dists)
  dist = jp.where(jp.isinf(dists[min_id]), -1.0, dists[min_id])
  id_ = jp.where(jp.isinf(dists[min_id]), -1, ids[min_id])

  return dist, id_

def ray_geom(
    size: jax.Array, pnt: jax.Array, vec: jax.Array, geomtype: GeomType
) -> jax.Array:
  """Returns the distance at which a ray intersects with a primitive geom.

  Args:
    size: geom size (1,), (2,), or (3,)
    pnt: ray origin point (3,)
    vec: ray direction    (3,)
    geomtype: type of geom

  Returns:
    dist: distance from ray origin to geom surface
  """
  return _RAY_FUNC[geomtype](size, pnt, vec)