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
"""Mesh processing."""

import itertools
from typing import Dict, Optional, Sequence, Tuple

import mujoco
# pylint: disable=g-importing-member
from mujoco.mjx._src.types import GeomType
from mujoco.mjx._src.types import Model
# pylint: enable=g-importing-member
import numpy as np
from scipy import spatial
import trimesh


_BOX_CORNERS = list(itertools.product((-1, 1), (-1, 1), (-1, 1)))
# pyformat: disable
# Rectangular box faces using a counter-clockwise winding order convention.
_BOX_FACES = [
    0, 4, 5, 1,  # left
    0, 2, 6, 4,  # bottom
    6, 7, 5, 4,  # front
    2, 3, 7, 6,  # right
    1, 5, 7, 3,  # top
    0, 1, 3, 2,  # back
]
# pyformat: enable
_MAX_HULL_FACE_VERTICES = 20
_CONVEX_CACHE: Dict[Tuple[int, int], Dict[str, np.ndarray]] = {}
_DERIVED_ARGS = [
    'geom_convex_face',
    'geom_convex_vert',
    'geom_convex_edge',
    'geom_convex_facenormal',
]
DERIVED = {(Model, d) for d in _DERIVED_ARGS}


def _box(size: np.ndarray):
  """Creates a mesh for a box with rectangular faces."""
  box_corners = np.array(_BOX_CORNERS)
  vert = box_corners * size.reshape(-1, 3)
  face = np.array([_BOX_FACES]).reshape(-1, 4)
  return vert, face


def _get_face_norm(vert: np.ndarray, face: np.ndarray) -> np.ndarray:
  """Calculates face normals given vertices and face indexes."""
  assert len(vert.shape) == 2 and len(face.shape) == 2, (
      f'vert and face should have dim of 2, got {len(vert.shape)} and '
      f'{len(face.shape)}'
  )
  face_vert = vert[face, :]
  # use CCW winding order convention
  edge0 = face_vert[:, 1, :] - face_vert[:, 0, :]
  edge1 = face_vert[:, -1, :] - face_vert[:, 0, :]
  face_norm = np.cross(edge0, edge1)
  face_norm = face_norm / np.linalg.norm(face_norm, axis=1).reshape((-1, 1))
  return face_norm


def _get_unique_edges(vert: np.ndarray, face: np.ndarray) -> np.ndarray:
  """Returns unique edges.

  Args:
    vert: (n_vert, 3) vertices
    face: (n_face, n_vert) face index array

  Returns:
    edges: tuples of vertex indexes for each edge
  """
  r_face = np.roll(face, 1, axis=1)
  edges = np.concatenate(np.array([face, r_face]).T)

  # do a first pass to remove duplicates
  edges.sort(axis=1)
  edges = np.unique(edges, axis=0)
  edges = edges[edges[:, 0] != edges[:, 1]]  # get rid of edges from padded face

  # get normalized edge directions
  edge_vert = vert.take(edges, axis=0)
  edge_dir = edge_vert[:, 0] - edge_vert[:, 1]
  norms = np.sqrt(np.sum(edge_dir**2, axis=1))
  edge_dir = edge_dir / norms.reshape((-1, 1))

  # get the first unique edge for all pairwise comparisons
  diff1 = edge_dir[:, None, :] - edge_dir[None, :, :]
  diff2 = edge_dir[:, None, :] + edge_dir[None, :, :]
  matches = (np.linalg.norm(diff1, axis=-1) < 1e-6) | (
      np.linalg.norm(diff2, axis=-1) < 1e-6
  )
  matches = np.tril(matches).sum(axis=-1)
  unique_edge_idx = np.where(matches == 1)[0]

  return edges[unique_edge_idx]


def _convex_hull_2d(points: np.ndarray, normal: np.ndarray) -> np.ndarray:
  """Calculates the convex hull for a set of points on a plane."""
  # project points onto the closest axis plane
  best_axis = np.abs(np.eye(3).dot(normal)).argmax()
  axis = np.eye(3)[best_axis]
  d = points.dot(axis).reshape((-1, 1))
  axis_points = points - d * axis
  axis_points = axis_points[:, list({0, 1, 2} - {best_axis})]

  # get the polygon face, and make the points ccw wrt the face normal
  c = spatial.ConvexHull(axis_points)
  order_ = np.where(axis.dot(normal) > 0, 1, -1)
  order_ *= np.where(best_axis == 1, -1, 1)
  hull_point_idx = c.vertices[::order_]
  assert (axis_points - c.points).sum() == 0

  return hull_point_idx


def _merge_coplanar(tm: trimesh.Trimesh) -> np.ndarray:
  """Merges coplanar facets."""
  if not tm.facets:
    return tm.faces.copy()  # no facets
  if not tm.faces.shape[0]:
    raise ValueError('Mesh has no faces.')

  # Get faces.
  face_idx = set(range(tm.faces.shape[0])) - set(np.concatenate(tm.facets))
  face_idx = np.array(list(face_idx))
  faces = tm.faces[face_idx] if face_idx.shape[0] > 0 else np.array([])

  # Get facets.
  facets = []
  for i, facet in enumerate(tm.facets):
    point_idx = np.unique(tm.faces[facet])
    points = tm.vertices[point_idx]
    normal = tm.facets_normal[i]

    # convert triangulated facet to a polygon
    hull_point_idx = _convex_hull_2d(points, normal)
    face = point_idx[hull_point_idx]

    # resize faces that exceed max polygon vertices
    every = face.shape[0] // _MAX_HULL_FACE_VERTICES + 1
    face = face[::every]
    facets.append(face)

  # Pad facets so that they can be stacked.
  max_len = max(f.shape[0] for f in facets) if facets else faces.shape[1]
  assert max_len <= _MAX_HULL_FACE_VERTICES
  for i, f in enumerate(facets):
    if f.shape[0] < max_len:
      f = np.pad(f, (0, max_len - f.shape[0]), 'edge')
    facets[i] = f

  if not faces.shape[0]:
    assert facets
    return np.array(facets)  # no faces, return facets

  # Merge faces and facets.
  faces = np.pad(faces, ((0, 0), (0, max_len - faces.shape[1])), 'edge')
  return np.concatenate([faces, facets])


def _get_faces_verts(
    m: mujoco.MjModel,
) -> Tuple[Sequence[np.ndarray], Sequence[np.ndarray]]:
  """Extracts mesh faces and vertices from MjModel."""
  verts, faces = [], []
  for i in range(m.nmesh):
    last = (i + 1) >= m.nmesh
    face_start = m.mesh_faceadr[i]
    face_end = m.mesh_faceadr[i + 1] if not last else m.mesh_face.shape[0]
    face = m.mesh_face[face_start:face_end]
    faces.append(face)

    vert_start = m.mesh_vertadr[i]
    vert_end = m.mesh_vertadr[i + 1] if not last else m.mesh_vert.shape[0]
    vert = m.mesh_vert[vert_start:vert_end]
    verts.append(vert)
  return verts, faces


def _geom_mesh_kwargs(
    vert: np.ndarray, face: np.ndarray
) -> Dict[str, np.ndarray]:
  """Generates convex mesh attributes for mjx.Model."""
  tm = trimesh.Trimesh(vertices=vert, faces=face)
  tm_convex = trimesh.convex.convex_hull(tm)
  vert = np.array(tm_convex.vertices)
  face = _merge_coplanar(tm_convex)
  return {
      'geom_convex_face': face,
      'geom_convex_vert': vert,
      'geom_convex_edge': _get_unique_edges(vert, face),
      'geom_convex_facenormal': _get_face_norm(vert, face),
  }


def get(m: mujoco.MjModel) -> Dict[str, Sequence[Optional[np.ndarray]]]:
  """Derives geom mesh attributes for mjx.Model from MjModel."""
  kwargs = {k: [] for k in _DERIVED_ARGS}
  verts, faces = _get_faces_verts(m)
  for geomid in range(m.ngeom):
    dataid = m.geom_dataid[geomid]
    typ = m.geom_type[geomid]
    if typ == GeomType.BOX:
      vert, face = _box(m.geom_size[geomid])
    elif dataid >= 0:
      vert, face = verts[dataid], faces[dataid]
    else:
      kwargs = {k: kwargs[k] + [None] for k in _DERIVED_ARGS}
      continue

    key = (hash(vert.data.tobytes()), hash(face.data.tobytes()))
    if key not in _CONVEX_CACHE:
      _CONVEX_CACHE[key] = _geom_mesh_kwargs(vert, face)

    kwargs = {k: kwargs[k] + [_CONVEX_CACHE[key][k]] for k in _DERIVED_ARGS}

  return kwargs
