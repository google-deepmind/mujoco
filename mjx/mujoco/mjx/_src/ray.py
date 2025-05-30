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
from jax import lax
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
  default_return_val = jp.array(-1.0, dtype=pnt.dtype)

  # Variables needed by multiple branches or lexically captured if not passed:
  # m, d, geomid, pnt, vec, hfid, size, nrow, ncol, default_return_val

  def _false_fn_vec_norm_ok(operand):
    # Unpack operand
    pnt_local_op, vec_local_orig_op, vec_norm_op = operand
    # Original logic:
    vec_local_norm = vec_local_orig_op / vec_norm_op

    def _false_fn_parallel_ok(operand_parallel):
      # Unpack operand_parallel
      pnt_local_p, vec_local_p, vec_norm_p = operand_parallel
      # Original logic:
      # Compute intersection with bounding box in XY plane
      # Note: size, nrow, ncol, hfid are from the outer scope of ray_hfield
      x0, y0 = -size[0], -size[1]
      x1, y1 = size[0], size[1]

      # Avoid division by zero
      vec_local_x_safe = jp.where(jp.abs(vec_local_p[0]) < 1e-10, jp.sign(vec_local_p[0]) * 1e-10 + 1e-10, vec_local_p[0])
      vec_local_y_safe = jp.where(jp.abs(vec_local_p[1]) < 1e-10, jp.sign(vec_local_p[1]) * 1e-10 + 1e-10, vec_local_p[1])

      tx0 = (x0 - pnt_local_p[0]) / vec_local_x_safe
      tx1 = (x1 - pnt_local_p[0]) / vec_local_x_safe
      ty0 = (y0 - pnt_local_p[1]) / vec_local_y_safe
      ty1 = (y1 - pnt_local_p[1]) / vec_local_y_safe

      tmin = jp.maximum(jp.minimum(tx0, tx1), jp.minimum(ty0, ty1))
      tmax = jp.minimum(jp.maximum(tx0, tx1), jp.maximum(ty0, ty1))

      def _false_fn_bbox_ok(operand_bbox):
        # Unpack operand_bbox
        ( pnt_local_b, vec_local_b, vec_norm_b, tmin_b, tmax_b,
          x0_b, y0_b # Needed by march_ray
        ) = operand_bbox
        # Original logic:
        t_init_loop = jp.maximum(tmin_b, 0.0)

        # hf_scale uses size, ncol, nrow from outer scope
        hf_scale = jp.array([2*size[0]/ncol, 2*size[1]/nrow, size[2], size[3]], dtype=pnt.dtype)


        def march_ray_local(t_arg, found_arg): # Renamed to avoid conflict
          p_march = pnt_local_b + t_arg * vec_local_b
          col_march = jp.clip((p_march[0] - x0_b) / (2 * size[0]) * ncol, 0, ncol-2)
          row_march = jp.clip((p_march[1] - y0_b) / (2 * size[1]) * nrow, 0, nrow-2)
          col_i_march = jp.floor(col_march).astype(jp.int32)
          row_i_march = jp.floor(row_march).astype(jp.int32)
          col_f_march = col_march - col_i_march
          row_f_march = row_march - row_i_march
          # hfid, m from outer scope
          idx00 = row_i_march * ncol + col_i_march
          idx01 = row_i_march * ncol + col_i_march + 1
          idx10 = (row_i_march + 1) * ncol + col_i_march
          idx11 = (row_i_march + 1) * ncol + col_i_march + 1
          h00 = m.hfield_data[hfid, idx00] * hf_scale[2]
          h01 = m.hfield_data[hfid, idx01] * hf_scale[2]
          h10 = m.hfield_data[hfid, idx10] * hf_scale[2]
          h11 = m.hfield_data[hfid, idx11] * hf_scale[2]
          h_interp = (h00 * (1-col_f_march)*(1-row_f_march) + h01*col_f_march*(1-row_f_march) +
                      h10*(1-col_f_march)*row_f_march + h11*col_f_march*row_f_march)
          intersection_march = (p_march[2] <= h_interp) & (vec_local_b[2] < 0) # Use &
          return t_arg, intersection_march

        abs_vec_local_z = jp.abs(vec_local_b[2])
        step_size_denominator = jp.where(abs_vec_local_z < 1e-10, 1e-10, abs_vec_local_z)
        step_size = jp.minimum(size[0] / ncol, size[1] / nrow) * 0.5 / step_size_denominator
        max_steps = 100

        init_val_loop = (t_init_loop, default_return_val, 0)

        def cond_loop_local(state_loop):
          curr_t, final_t, iter_v = state_loop
          return (iter_v < max_steps) & jp.equal(final_t, default_return_val) & (curr_t <= tmax_b)

        def body_loop_local(state_loop):
          curr_t, final_t, iter_v = state_loop
          _, hit = march_ray_local(curr_t, False)
          _vec_norm = vec_norm_b.astype(curr_t.dtype)
          new_final_t = jp.where(hit, curr_t * _vec_norm, final_t)
          new_curr_t = curr_t + step_size
          return (new_curr_t, new_final_t, iter_v + 1)

        final_state_loop = lax.while_loop(cond_loop_local, body_loop_local, init_val_loop)
        return jp.array(final_state_loop[1], dtype=pnt.dtype)

      # Operand for bbox condition:
      operand_bbox_cond = (pnt_local_p, vec_local_p, vec_norm_p, tmin, tmax, x0, y0)
      return lax.cond(
          (tmin > tmax) | (tmax < 0),
          lambda _: default_return_val,
          _false_fn_bbox_ok,
          operand_bbox_cond
      )

    # Operand for parallel condition:
    operand_parallel_cond = (pnt_local_op, vec_local_norm, vec_norm_op)
    return lax.cond(
        jp.abs(vec_local_norm[2]) < 1e-10,
        lambda _: default_return_val,
        _false_fn_parallel_ok,
        operand_parallel_cond
    )

  # Operand for vec_norm condition:
  operand_vec_norm_cond = (pnt_local, vec_local, vec_norm) # vec_local is pre-normalization here
  return lax.cond(
      vec_norm < 1e-10,
      lambda _: default_return_val,
      _false_fn_vec_norm_ok,
      operand_vec_norm_cond
  )


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
    GeomType.HFIELD: lambda m, d, *args: _ray_hfield_wrapper(m, d, *args),  # Added for height field
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

    if geom_type == GeomType.MESH:
      dist, id_ = fn(m, id_, *args)  # Pass m for MESH
    elif geom_type == GeomType.HFIELD:
      dist, id_ = fn(m, d, id_, *args)  # Pass m and d for HFIELD
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
