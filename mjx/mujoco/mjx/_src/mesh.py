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

import collections
import itertools
from typing import Tuple, Union
import warnings

import jax
from jax import numpy as jp
import mujoco
from mujoco.mjx._src import math
# pylint: disable=g-importing-member
from mujoco.mjx._src.collision_types import ConvexInfo
from mujoco.mjx._src.collision_types import GeomInfo
from mujoco.mjx._src.collision_types import HFieldInfo
from mujoco.mjx._src.types import ConvexMesh
from mujoco.mjx._src.types import Model
# pylint: enable=g-importing-member
import numpy as np
from scipy import spatial
import trimesh


_MAX_HULL_FACE_VERTICES = 20


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


def _get_edge_normals(
    face: np.ndarray, face_norm: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
  """Returns face edges and face edge normals."""
  # get face edges and scatter the face norms
  r_face = np.roll(face, 1, axis=1)
  face_edge = np.array([face, r_face]).transpose((1, 2, 0))
  face_edge.sort(axis=2)
  face_edge_flat = np.concatenate(face_edge)
  edge_face_idx = np.repeat(np.arange(face.shape[0]), face.shape[1])
  edge_face_norm = face_norm[edge_face_idx]

  # get the edge normals associated with each edge
  edge_map_list = collections.defaultdict(list)
  for i in range(face_edge_flat.shape[0]):
    if face_edge_flat[i][0] == face_edge_flat[i][1]:
      continue
    edge_map_list[tuple(face_edge_flat[i])].append(edge_face_norm[i])

  edges, edge_face_normals = [], []
  for k, v in edge_map_list.items():
    v = np.array(v)
    if len(v) > 2:
      # Meshes can be of poor quality and contain edges adjacent to more than
      # two faces. We take the first two unique face normals.
      v = np.unique(v, axis=0)[:2]
    elif len(v) == 1:
      # Some edges are either degenerate or _MAX_HULL_FACE_VERTICES was hit
      # and face vertices were down sampled. In either case, we ignore these
      # edges.
      continue
    edges.append(k)
    edge_face_normals.append(v)

  return np.array(edges), np.array(edge_face_normals)


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


def _merge_coplanar(
    m: Union[mujoco.MjModel, Model], tm: trimesh.Trimesh, meshid: int
) -> np.ndarray:
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
    if face.shape[0] > _MAX_HULL_FACE_VERTICES:
      name = m.names[m.name_meshadr[meshid] :]
      name = name[: name.find(b'\x00')].decode('utf-8')
      warnings.warn(
          f'Mesh "{name}" has a coplanar face with more than '
          f'{_MAX_HULL_FACE_VERTICES} vertices. This may lead to performance '
          'issues and inaccuracies in collision detection. Consider '
          'decimating the mesh.'
      )
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


def box(info: GeomInfo) -> ConvexInfo:
  """Creates a box with rectangular faces."""
  vert = np.array(
      list(itertools.product((-1, 1), (-1, 1), (-1, 1))), dtype=float
  )
  # pyformat: disable
  # rectangular box faces using a counter-clockwise winding order convention:
  face = np.array(
      [
          0, 4, 5, 1,  # left
          0, 2, 6, 4,  # bottom
          6, 7, 5, 4,  # front
          2, 3, 7, 6,  # right
          1, 5, 7, 3,  # top
          0, 1, 3, 2,  # back
      ]
  ).reshape((-1, 4))
  # pyformat: enable
  face_normal = _get_face_norm(vert, face)
  edge, edge_face_normal = _get_edge_normals(face, face_normal)
  face = vert[face]  # materialize full nface x nvert matrix

  c = ConvexInfo(
      info.pos,
      info.mat,
      info.size,
      vert,
      face,
      face_normal,
      edge,
      edge_face_normal,
  )
  c = jax.tree_util.tree_map(jp.array, c)
  vert = jax.vmap(jp.multiply, in_axes=(None, 0))(c.vert, info.size)
  face = jax.vmap(jp.multiply, in_axes=(None, 0))(c.face, info.size)
  c = c.replace(vert=vert, face=face)

  return c


def convex(m: Union[mujoco.MjModel, Model], data_id: int) -> ConvexMesh:
  """Processes a mesh for use in convex collision algorithms.

  Args:
    m: an MJX model
    data_id: the mesh id to process

  Returns:
    a convex mesh
  """
  vert_beg = m.mesh_vertadr[data_id]
  vert_end = m.mesh_vertadr[data_id + 1] if data_id < m.nmesh - 1 else None
  vert = m.mesh_vert[vert_beg:vert_end]

  graphadr = m.mesh_graphadr[data_id]
  graph = m.mesh_graph[graphadr:]
  graph_idx = 0

  numvert, numface = graph[0], graph[1]
  graph_idx += 2

  # skip vert_edgeadr  (numvert,)
  graph_idx += numvert
  vert_globalid = graph[graph_idx : graph_idx + numvert]
  graph_idx += numvert

  # skip edge_localid  (numvert, 3)
  graph_idx += numvert + 3 * numface
  face_globalid = graph[graph_idx : graph_idx + 3 * numface].reshape((-1, 3))

  vert = vert[vert_globalid]
  vertex_map = dict(zip(vert_globalid, np.arange(vert_globalid.shape[0])))
  face = np.vectorize(vertex_map.get)(face_globalid)

  tm_convex = trimesh.Trimesh(vertices=vert, faces=face)
  vert = np.array(tm_convex.vertices)
  face = _merge_coplanar(m, tm_convex, data_id)
  face_normal = _get_face_norm(vert, face)
  edge, edge_face_normal = _get_edge_normals(face, face_normal)
  face = vert[face]  # materialize full nface x nvert matrix

  c = ConvexMesh(
      vert,
      face,
      face_normal,
      edge,
      edge_face_normal,
  )

  return jax.tree_util.tree_map(jp.array, c)


def hfield_prism(vert: jax.Array) -> ConvexInfo:
  """Builds a hfield prism."""
  # The first 3 vertices define the bottom triangle, and the next 3 vertices
  # define the top triangle. The remaining triangles define the side of the
  # prism.
  face = np.array([
      [0, 1, 2, 0],  # bottom
      [3, 4, 5, 3],  # top
      [0, 3, 5, 1],
      [0, 2, 4, 3],
      [2, 1, 5, 4],
  ])
  edges = np.array([
      # bottom
      [0, 1],
      [1, 2],
      [0, 2],
      # top
      [3, 4],
      [3, 5],
      [4, 5],
      # sides
      [0, 3],
      [1, 5],
      [2, 4],
  ])
  edge_face_norm = np.array([
      # bottom
      [0, 2],
      [0, 4],
      [0, 3],
      # top
      [1, 3],
      [1, 2],
      [1, 4],
      # sides
      [2, 3],
      [2, 4],
      [3, 4],
  ])

  def get_face_norm(face):
    # use ccw winding order convention, and avoid using the last vertex
    edge0 = face[2, :] - face[1, :]
    edge1 = face[0, :] - face[1, :]
    return math.normalize(jp.cross(edge0, edge1))

  centroid = jp.mean(vert, axis=0)
  vert = vert - centroid
  face = vert[face]
  face_norm = jax.vmap(get_face_norm)(face)

  c = ConvexInfo(
      centroid,
      jp.eye(3, dtype=float),
      jp.ones(3),
      vert,
      face,
      face_norm,
      edges,
      face_norm[edge_face_norm],
  )

  return jax.tree_util.tree_map(jp.array, c)


def hfield(m: Union[mujoco.MjModel, Model], data_id: int) -> HFieldInfo:
  adr = m.hfield_adr[data_id]
  nrow, ncol = m.hfield_nrow[data_id], m.hfield_ncol[data_id]
  h = HFieldInfo(
      jp.zeros(3, dtype=float),
      jp.eye(3, dtype=float),
      m.hfield_size[data_id],
      nrow,
      ncol,
      m.hfield_data[adr : adr + nrow * ncol].reshape((ncol, nrow), order='F'),
  )
  return h
