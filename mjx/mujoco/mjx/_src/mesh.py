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
import dataclasses
import itertools
from typing import Dict, List, Optional, Sequence, Tuple
import warnings

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
    'geom_convex_edge_dir',
    'geom_convex_facenormal',
    'geom_convex_face_edge',
    'geom_convex_face_edge_normal',
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


def _get_unique_edge_dir(vert: np.ndarray, face: np.ndarray) -> np.ndarray:
  """Returns unique edge directions.

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


def _get_face_edge_normals(
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

  edge_map = {}
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
    edge_map[k] = v

  # for each face, list the edge normals
  face_edge_normal = []
  for face_idx in range(face_edge.shape[0]):
    normals = []
    for edge in face_edge[face_idx]:
      k = tuple(edge)
      normals.append(edge_map.get(k, np.zeros((2, 3))))
    face_edge_normal.append(np.array(normals))
  face_edge_normal = np.array(face_edge_normal)

  return face_edge, face_edge_normal


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


@dataclasses.dataclass
class MeshInfo:
  name: str
  vert: np.ndarray
  face: np.ndarray
  convex_vert: Optional[np.ndarray]
  convex_face: Optional[np.ndarray]


def _merge_coplanar(tm: trimesh.Trimesh, mesh_info: MeshInfo) -> np.ndarray:
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
      warnings.warn(
          f'Mesh "{mesh_info.name}" has a coplanar face with more than'
          f' {_MAX_HULL_FACE_VERTICES} vertices. This may lead to performance '
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


def _mesh_info(
    m: mujoco.MjModel,
) -> List[MeshInfo]:
  """Extracts mesh info from MjModel."""
  mesh_infos = []
  for i in range(m.nmesh):
    name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH.value, i)

    last = (i + 1) >= m.nmesh
    face_start = m.mesh_faceadr[i]
    face_end = m.mesh_faceadr[i + 1] if not last else m.mesh_face.shape[0]
    face = m.mesh_face[face_start:face_end]

    vert_start = m.mesh_vertadr[i]
    vert_end = m.mesh_vertadr[i + 1] if not last else m.mesh_vert.shape[0]
    vert = m.mesh_vert[vert_start:vert_end]

    graphadr = m.mesh_graphadr[i]
    if graphadr < 0:
      mesh_infos.append(MeshInfo(name, vert, face, None, None))
      continue

    graph = m.mesh_graph[graphadr:]
    numvert, numface = graph[0], graph[1]

    # unused vert_edgeadr
    # vert_edgeadr = graph[2 : numvert + 2]
    last_idx = numvert + 2

    vert_globalid = graph[last_idx : last_idx + numvert]
    last_idx += numvert

    # unused edge_localid
    # edge_localid = graph[last_idx : last_idx + numvert + 3 * numface]
    last_idx += numvert + 3 * numface

    face_globalid = graph[last_idx : last_idx + 3 * numface]
    face_globalid = face_globalid.reshape((numface, 3))

    convex_vert = vert[vert_globalid]
    vertex_map = dict(zip(vert_globalid, np.arange(vert_globalid.shape[0])))
    convex_face = np.vectorize(vertex_map.get)(face_globalid)
    mesh_infos.append(MeshInfo(name, vert, face, convex_vert, convex_face))

  return mesh_infos


def _geom_mesh_kwargs(
    mesh_info: MeshInfo,
) -> Dict[str, np.ndarray]:
  """Generates convex mesh attributes for mjx.Model."""
  tm_convex = trimesh.Trimesh(
      vertices=mesh_info.convex_vert, faces=mesh_info.convex_face
  )
  vert = np.array(tm_convex.vertices)
  face = _merge_coplanar(tm_convex, mesh_info)
  facenormal = _get_face_norm(vert, face)
  face_edge, face_edge_normal = _get_face_edge_normals(face, facenormal)
  return {
      'geom_convex_face': vert[face],
      'geom_convex_face_vert_idx': face,
      'geom_convex_vert': vert,
      'geom_convex_edge_dir': _get_unique_edge_dir(vert, face),
      'geom_convex_facenormal': facenormal,
      'geom_convex_face_edge': face_edge,
      'geom_convex_face_edge_normal': face_edge_normal,
  }


def get(m: mujoco.MjModel) -> Dict[str, Sequence[Optional[np.ndarray]]]:
  """Derives geom mesh attributes for mjx.Model from MjModel."""
  kwargs = {k: [] for k in _DERIVED_ARGS}
  mesh_infos = _mesh_info(m)
  geom_con = m.geom_conaffinity | m.geom_contype
  for geomid in range(m.ngeom):
    mesh_info = None
    dataid = m.geom_dataid[geomid]
    if not geom_con[geomid]:
      # ignore visual-only meshes
      kwargs = {k: kwargs[k] + [None] for k in _DERIVED_ARGS}
      continue
    elif m.geom_type[geomid] == GeomType.BOX:
      vert, face = _box(m.geom_size[geomid])
      mesh_info = MeshInfo(
          name='box',
          vert=vert,
          face=face,
          convex_vert=vert,
          convex_face=face,
      )
    elif dataid < 0:
      kwargs = {k: kwargs[k] + [None] for k in _DERIVED_ARGS}
      continue

    mesh_info = mesh_info or mesh_infos[dataid]
    vert, face = mesh_info.vert, mesh_info.face
    key = (hash(vert.data.tobytes()), hash(face.data.tobytes()))
    if key not in _CONVEX_CACHE:
      _CONVEX_CACHE[key] = _geom_mesh_kwargs(mesh_info)

    kwargs = {k: kwargs[k] + [_CONVEX_CACHE[key][k]] for k in _DERIVED_ARGS}

  return kwargs
