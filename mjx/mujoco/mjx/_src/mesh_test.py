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
"""Tests for mesh.py."""

from absl.testing import absltest
from mujoco.mjx._src import mesh
import numpy as np
import trimesh


class MeshTest(absltest.TestCase):

  def test_pyramid(self):
    """Tests that a triangulated pyramid converts to merged coplanar faces."""
    vert = np.array([
        [-0.025, 0.05, 0.05],
        [-0.025, -0.05, -0.05],
        [-0.025, -0.05, 0.05],
        [-0.025, 0.05, -0.05],
        [0.075, 0.0, 0.0],
    ])
    face = np.array(
        [[0, 1, 2], [0, 3, 1], [0, 4, 3], [0, 2, 4], [2, 1, 4], [1, 3, 4]]
    )
    tm = trimesh.Trimesh(vertices=vert, faces=face)
    tm_convex = trimesh.convex.convex_hull(tm)
    convex_vert = np.array(tm_convex.vertices)
    convex_face = mesh._merge_coplanar(None, tm_convex, 0)

    # get index of vertices in h['geom_convex_vert'] for vertices in vert
    dist = np.repeat(vert, vert.shape[0], axis=0) - np.tile(
        convex_vert, (vert.shape[0], 1)
    )
    dist = (dist**2).sum(axis=1).reshape((vert.shape[0], -1))
    vidx = np.argmin(dist, axis=0)

    # check verts
    np.testing.assert_array_equal(convex_vert, vert[vidx])

    # check face vertices
    map_ = {v: k for k, v in enumerate(vidx)}
    h_face = np.vectorize(map_.get)(convex_face)
    face_verts = sorted([tuple(sorted(set(s))) for s in h_face.tolist()])
    expected_face_verts = sorted(
        [(0, 3, 4), (1, 3, 4), (0, 2, 4), (0, 1, 2, 3), (1, 2, 4)]
    )
    self.assertSequenceEqual(
        face_verts,
        expected_face_verts,
    )

    # face normals
    face_normal = mesh._get_face_norm(convex_vert, convex_face)
    self.assertEqual(face_normal.shape, (5, 3))

    # face edges
    edges, edge_normal = mesh._get_edge_normals(convex_face, face_normal)
    edges = np.vectorize(map_.get)(edges)
    mask = edges[:, 0] != edges[:, 1]
    edges = edges[mask]
    sort_col_idx = np.argsort(edges, axis=1)
    edges = np.take_along_axis(edges, sort_col_idx, axis=1)
    sort_row_idx = np.lexsort((edges[:, 1], edges[:, 0]))
    edges = edges[sort_row_idx]
    np.testing.assert_array_equal(
        edges,
        np.array([
            [0, 2],
            [0, 3],
            [0, 4],
            [1, 2],
            [1, 3],
            [1, 4],
            [2, 4],
            [3, 4],
        ]),
    )

    # face edge normals
    edge_normal = edge_normal[mask]
    edge_normal = np.take_along_axis(
        edge_normal, sort_col_idx[..., None], axis=1
    )
    edge_normal = edge_normal[sort_row_idx]
    edge_normal_02 = np.array([[0.4472136, -0.0, 0.89442719], [-1.0, 0.0, 0.0]])
    np.testing.assert_array_almost_equal(
        edge_normal[:1],
        np.array([edge_normal_02]),
    )


class ConvexHull2DTest(absltest.TestCase):

  def test_convex_hull_2d_axis1(self):
    """Tests for the correct winding order of a polgyon with +y normal."""
    pts = np.array([
        [-0.04634297, -0.06652775, 0.05853534],
        [-0.01877651, -0.08309858, -0.05236476],
        [0.02362804, -0.08010745, 0.05499557],
        [0.04066505, -0.09034877, -0.01354446],
        [-0.07255043, -0.06837638, -0.00781699],
    ])
    normal = np.array([-0.18467607, -0.97768016, 0.10018111])
    idx = mesh._convex_hull_2d(pts, normal)
    expected = np.cross(pts[idx][1] - pts[idx][0], pts[idx][2] - pts[idx][0])
    expected /= np.linalg.norm(expected)
    np.testing.assert_array_almost_equal(normal, expected)

  def test_convex_hull_2d_axis2(self):
    """Tests for the correct winding order for a polgyon with +z normal."""
    pts = np.array([
        [0.08607829, -0.03881998, -0.03291714],
        [-0.01877651, -0.08309858, -0.05236476],
        [0.05470364, 0.00027677, -0.08371042],
        [-0.01010019, -0.02708892, -0.0957297],
        [0.04066505, -0.09034877, -0.01354446],
    ])
    normal = np.array([0.3839915, -0.60171936, -0.70034587])
    idx = mesh._convex_hull_2d(pts, normal)
    expected = np.cross(pts[idx][1] - pts[idx][0], pts[idx][2] - pts[idx][0])
    expected /= np.linalg.norm(expected)
    np.testing.assert_array_almost_equal(normal, expected)


if __name__ == '__main__':
  absltest.main()
