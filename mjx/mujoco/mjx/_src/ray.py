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

from typing import Sequence, Tuple

import jax
from jax import numpy as jp
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

  x0, x1 = (-b - det_2) / a, (-b + det_2) / a
  x0 = jp.where((det < mujoco.mjMINVAL) | (x0 < 0), jp.inf, x0)
  x1 = jp.where((det < mujoco.mjMINVAL) | (x1 < 0), jp.inf, x1)

  return x0, x1


def _ray_plane(
    size: jax.Array,
    pnt: jax.Array,
    vec: jax.Array,
) -> jax.Array:
  """Returns the distance at which a ray intersects with a plane."""
  x = -pnt[2] / vec[2]

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
  s = 1 / jp.square(size)

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
  x = jp.concatenate([(size - pnt) / vec, (-size - pnt) / vec])

  # intersection with face
  p0 = pnt[iface[:, 0]] + x * vec[iface[:, 0]]
  p1 = pnt[iface[:, 1]] + x * vec[iface[:, 1]]
  valid = jp.abs(p0) <= size[iface[:, 0]]
  valid &= jp.abs(p1) <= size[iface[:, 1]]
  valid &= x >= 0

  return jp.min(jp.where(valid, x, jp.inf))


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

  t0 = (A[1, 1] * b[0] - A[1, 0] * b[1]) / det
  t1 = (-A[0, 1] * b[0] + A[0, 0] * b[1]) / det
  valid = (t0 >= 0) & (t1 >= 0) & (t0 + t1 <= 1)

  # intersect ray with plane of triangle
  nrm = jp.cross(vert[0] - vert[2], vert[1] - vert[2])
  dist = jp.dot(vert[2] - pnt, nrm) / jp.dot(vec, nrm)
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


def ray_hfield(m: Model, d: Data, geomid: int, pnt: jax.Array, vec: jax.Array) -> jax.Array:
  """Ray intersection with height field.

  Args:
    m: MuJoCo model
    d: MuJoCo data
    geomid: ID of the height field geom
    pnt: Ray origin (3D)
    vec: Ray direction (3D)

  Returns:
    Nearest distance or -1 if no intersection
  """
  # Get hfield data
  hfid = m.geom_dataid[geomid]
  size = m.hfield_size[hfid]
  nrow = m.hfield_nrow[hfid]
  ncol = m.hfield_ncol[hfid]

  # Get geom position and orientation
  pos = d.geom_xpos[geomid]
  mat = d.geom_xmat[geomid].reshape(3, 3)
  # Express ray in local coordinates
  vec_local = mat.T @ vec
  pnt_local = mat.T @ (pnt - pos)

  # Normalize ray direction
  vec_norm = jp.linalg.norm(vec_local)
  if vec_norm < 1e-10:
    return jp.array(-1.0)
  vec_local = vec_local / vec_norm

  # X and Y are the horizontal dimensions of the height field
  # (corresponding to rows and columns of the height data)
  # Z is height

  # Check if ray is parallel to XY plane
  if jp.abs(vec_local[2]) < 1e-10:
    return jp.array(-1.0)

  # Compute intersection with bounding box in XY plane
  x0, y0 = -size[0], -size[1]  # bottom left in local coords
  x1, y1 = size[0], size[1]    # top right in local coords

  # Compute intersection with XY bounding box
  tx0 = (x0 - pnt_local[0]) / (vec_local[0] + 1e-10)
  tx1 = (x1 - pnt_local[0]) / (vec_local[0] + 1e-10)
  ty0 = (y0 - pnt_local[1]) / (vec_local[1] + 1e-10)
  ty1 = (y1 - pnt_local[1]) / (vec_local[1] + 1e-10)

  # Get entry and exit times
  tmin = jp.maximum(jp.minimum(tx0, tx1), jp.minimum(ty0, ty1))
  tmax = jp.minimum(jp.maximum(tx0, tx1), jp.maximum(ty0, ty1))

  # No intersection with bounding box
  if tmin > tmax or tmax < 0:
    return jp.array(-1.0)

  # Ensure we're not starting inside the height field
  t = jp.maximum(tmin, 0.0)

  # Get hfield data scaling
  hf_size = m.hfield_size[hfid]
  hf_scale = jp.array([2*size[0]/ncol, 2*size[1]/nrow, 1, 1])

  # Compute ray-heightfield intersection using a ray marching algorithm
  def march_ray(t, found):
    # Get current intersection point
    p = pnt_local + t * vec_local

    # Convert to hfield grid coordinates
    col = jp.clip((p[0] - x0) / (2 * size[0]) * ncol, 0, ncol-2)
    row = jp.clip((p[1] - y0) / (2 * size[1]) * nrow, 0, nrow-2)

    # Get integer coordinates
    col_i = jp.floor(col).astype(jp.int32)
    row_i = jp.floor(row).astype(jp.int32)

    # Get fractional parts for bilinear interpolation
    col_f = col - col_i
    row_f = row - row_i

    # Get heights at the four corners
    h00 = m.hfield_data[hfid, row_i * ncol + col_i] * hf_scale[2]
    h01 = m.hfield_data[hfid, row_i * ncol + col_i + 1] * hf_scale[2]
    h10 = m.hfield_data[hfid, (row_i + 1) * ncol + col_i] * hf_scale[2]
    h11 = m.hfield_data[hfid, (row_i + 1) * ncol + col_i + 1] * hf_scale[2]
    # Bilinear interpolation
    h = (h00 * (1 - col_f) * (1 - row_f) +
         h01 * col_f * (1 - row_f) +
         h10 * (1 - col_f) * row_f +
         h11 * col_f * row_f)

    # Check if ray passes below the height at this point
    intersection = p[2] <= h and vec_local[2] < 0

    # Binary search refinement could be added here for more precision

    return t, intersection

  # Initial ray marching with fixed step size
  step_size = jp.minimum(size[0] / ncol, size[1] / nrow) * 0.5 / jp.abs(vec_local[2])
  max_steps = 100  # Prevent infinite loops

  t_final = -1.0
  for _ in range(max_steps):
    t, hit = march_ray(t, False)
    if hit:
      t_final = t * vec_norm  # Scale back to original ray length
      break
    t += step_size
    # Exit if we've gone beyond the bounding box
    if t > tmax:
      break
  return jp.array(t_final)


def _ray_hfield_wrapper(
    m: Model,
    d: Data,
    geom_id: np.ndarray,
    unused_size: jax.Array,
    pnt: jax.Array,
    vec: jax.Array,
) -> Tuple[jax.Array, jax.Array]:
  """Wrapper for ray_hfield to match the expected interface for _RAY_FUNC."""
  dists, ids = [], []
  for i, id_ in enumerate(geom_id):
    dist = ray_hfield(m, d, id_, pnt[i], vec[i])
    dist = jp.reshape(dist, (1,))  # Reshape to match expected format
    dists.append(dist)
    ids.append(jp.array([id_]))

  dists = jp.concatenate(dists)
  ids = jp.concatenate(ids)
  return dists, ids


_RAY_FUNC = {
    GeomType.PLANE: _ray_plane,
    GeomType.SPHERE: _ray_sphere,
    GeomType.CAPSULE: _ray_capsule,
    GeomType.ELLIPSOID: _ray_ellipsoid,
    GeomType.BOX: _ray_box,
    GeomType.MESH: _ray_mesh,
    GeomType.HFIELD: lambda *args: _ray_hfield_wrapper(m, d, *args),  # Added for height field
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

    args = m.geom_size[id_], geom_pnts[id_], geom_vecs[id_]

    if geom_type in (GeomType.MESH, GeomType.HFIELD):
      dist, id_ = fn(id_, *args)
    else:
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
  # Note: ray_geom doesn't handle height fields. For height fields, use ray() instead
  if geomtype == GeomType.HFIELD:
    raise ValueError("ray_geom doesn't support height fields. Use ray() instead.")
  return _RAY_FUNC[geomtype](size, pnt, vec)
