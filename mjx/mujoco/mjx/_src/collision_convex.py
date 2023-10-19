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
"""Convex collisions."""

from typing import Tuple

import jax
from jax import numpy as jp
from mujoco.mjx._src import math
# pylint: disable=g-importing-member
from mujoco.mjx._src.collision_base import Contact
from mujoco.mjx._src.collision_base import GeomInfo
# pylint: enable=g-importing-member


def _closest_segment_point_plane(
    a: jax.Array, b: jax.Array, p0: jax.Array, plane_normal: jax.Array
) -> jax.Array:
  """Gets the closest point between a line segment and a plane.

  Args:
    a: first line segment point
    b: second line segment point
    p0: point on plane
    plane_normal: plane normal

  Returns:
    closest point between the line segment and the plane
  """
  # Parametrize a line segment as S(t) = a + t * (b - a), plug it into the plane
  # equation dot(n, S(t)) - d = 0, then solve for t to get the line-plane
  # intersection. We then clip t to be in [0, 1] to be on the line segment.
  n = plane_normal
  d = jp.sum(p0 * n)  # shortest distance from origin to plane
  denom = jp.sum(n * (b - a))
  t = (d - jp.sum(n * a)) / (denom + 1e-6 * (denom == 0.0))
  t = jp.clip(t, 0, 1)
  segment_point = a + t * (b - a)

  return segment_point


def _closest_triangle_point(
    p0: jax.Array, p1: jax.Array, p2: jax.Array, pt: jax.Array
) -> jax.Array:
  """Gets the closest point between a triangle and a point in space.

  Args:
    p0: triangle point
    p1: triangle point
    p2: triangle point
    pt: point to test

  Returns:
    closest point on the triangle w.r.t point pt
  """
  # Parametrize the triangle s.t. a point inside the triangle is
  # Q = p0 + u * e0 + v * e1, when 0 <= u <= 1, 0 <= v <= 1, and
  # 0 <= u + v <= 1. Let e0 = (p1 - p0) and e1 = (p2 - p0).
  # We analytically minimize the distance between the point pt and Q.
  e0 = p1 - p0
  e1 = p2 - p0
  a = e0.dot(e0)
  b = e0.dot(e1)
  c = e1.dot(e1)
  d = pt - p0
  # The determinant is 0 only if the angle between e1 and e0 is 0
  # (i.e. the triangle has overlapping lines).
  det = a * c - b * b
  u = (c * e0.dot(d) - b * e1.dot(d)) / det
  v = (-b * e0.dot(d) + a * e1.dot(d)) / det
  inside = (0 <= u) & (u <= 1) & (0 <= v) & (v <= 1) & (u + v <= 1)
  closest_p = p0 + u * e0 + v * e1
  d0 = (closest_p - pt).dot(closest_p - pt)

  # If the closest point is outside the triangle, it must be on an edge, so we
  # check each triangle edge for a closest point to the point pt.
  closest_p1, d1 = math.closest_segment_point_and_dist(p0, p1, pt)
  closest_p = jp.where((d0 < d1) & inside, closest_p, closest_p1)
  min_d = jp.where((d0 < d1) & inside, d0, d1)

  closest_p2, d2 = math.closest_segment_point_and_dist(p1, p2, pt)
  closest_p = jp.where(d2 < min_d, closest_p2, closest_p)
  min_d = jp.minimum(min_d, d2)

  closest_p3, d3 = math.closest_segment_point_and_dist(p2, p0, pt)
  closest_p = jp.where(d3 < min_d, closest_p3, closest_p)

  return closest_p


def _closest_segment_triangle_points(
    a: jax.Array,
    b: jax.Array,
    p0: jax.Array,
    p1: jax.Array,
    p2: jax.Array,
    triangle_normal: jax.Array,
) -> Tuple[jax.Array, jax.Array]:
  """Gets the closest points between a line segment and triangle.

  Args:
    a: first line segment point
    b: second line segment point
    p0: triangle point
    p1: triangle point
    p2: triangle point
    triangle_normal: normal of triangle

  Returns:
    closest point on the triangle w.r.t the line segment
  """
  # The closest triangle point is either on the edge or within the triangle.
  # First check triangle edges for the closest point.
  # TODO(robotics-simulation): consider vmapping over closest point functions
  seg_pt1, tri_pt1 = math.closest_segment_to_segment_points(a, b, p0, p1)
  d1 = (seg_pt1 - tri_pt1).dot(seg_pt1 - tri_pt1)
  seg_pt2, tri_pt2 = math.closest_segment_to_segment_points(a, b, p1, p2)
  d2 = (seg_pt2 - tri_pt2).dot(seg_pt2 - tri_pt2)
  seg_pt3, tri_pt3 = math.closest_segment_to_segment_points(a, b, p0, p2)
  d3 = (seg_pt3 - tri_pt3).dot(seg_pt3 - tri_pt3)

  # Next, handle the case where the closest triangle point is inside the
  # triangle. Either the line segment intersects the triangle or a segment
  # endpoint is closest to a point inside the triangle.
  seg_pt4 = _closest_segment_point_plane(a, b, p0, triangle_normal)
  tri_pt4 = _closest_triangle_point(p0, p1, p2, seg_pt4)
  d4 = (seg_pt4 - tri_pt4).dot(seg_pt4 - tri_pt4)

  # Get the point with minimum distance from the line segment point to the
  # triangle point.
  distance = jp.array([[d1, d2, d3, d4]])
  min_dist = jp.amin(distance)
  mask = (distance == min_dist).T
  seg_pt = jp.array([seg_pt1, seg_pt2, seg_pt3, seg_pt4]) * mask
  tri_pt = jp.array([tri_pt1, tri_pt2, tri_pt3, tri_pt4]) * mask
  seg_pt = jp.sum(seg_pt, axis=0) / jp.sum(mask)
  tri_pt = jp.sum(tri_pt, axis=0) / jp.sum(mask)

  return seg_pt, tri_pt


def _manifold_points(
    poly: jax.Array, poly_mask: jax.Array, poly_norm: jax.Array
) -> jax.Array:
  """Chooses four points on the polygon with approximately maximal area."""
  dist_mask = jp.where(poly_mask, 0.0, -1e6)
  a_idx = jp.argmax(dist_mask)
  a = poly[a_idx]
  # choose point b furthest from a
  b_idx = (((a - poly) ** 2).sum(axis=1) + dist_mask).argmax()
  b = poly[b_idx]
  # choose point c furthest along the axis orthogonal to (a-b)
  ab = jp.cross(poly_norm, a - b)
  ap = a - poly
  c_idx = (jp.abs(ap.dot(ab)) + dist_mask).argmax()
  c = poly[c_idx]
  # choose point d furthest from the other two triangle edges
  ac = jp.cross(poly_norm, a - c)
  bc = jp.cross(poly_norm, b - c)
  bp = b - poly
  dist_bp = jp.abs(bp.dot(bc)) + dist_mask
  dist_ap = jp.abs(ap.dot(ac)) + dist_mask
  d_idx = jp.concatenate([dist_bp, dist_ap]).argmax() % poly.shape[0]
  return jp.array([a_idx, b_idx, c_idx, d_idx])


def _project_pt_onto_plane(
    pt: jax.Array, plane_pt: jax.Array, plane_normal: jax.Array
) -> jax.Array:
  """Projects a point onto a plane along the plane normal."""
  dist = (pt - plane_pt).dot(plane_normal)
  return pt - dist * plane_normal


def _project_poly_onto_plane(
    poly: jax.Array, plane_pt: jax.Array, plane_normal: jax.Array
) -> jax.Array:
  """Projects a polygon onto a plane using the plane normal."""
  return jax.vmap(_project_pt_onto_plane, in_axes=[0, None, None])(
      poly, plane_pt, math.normalize(plane_normal)
  )


def _project_poly_onto_poly_plane(
    poly1: jax.Array, norm1: jax.Array, poly2: jax.Array, norm2: jax.Array
) -> jax.Array:
  """Projects poly1 onto the poly2 plane along poly1's normal."""
  d = poly2[0].dot(norm2)
  denom = norm1.dot(norm2)
  t = (d - poly1.dot(norm2)) / (denom + 1e-6 * (denom == 0.0))
  new_poly = poly1 + t.reshape(-1, 1) * norm1
  return new_poly


def _point_in_front_of_plane(
    plane_pt: jax.Array, plane_normal: jax.Array, pt: jax.Array
) -> jax.Array:
  """Checks if a point is strictly in front of a plane."""
  return (pt - plane_pt).dot(plane_normal) > 1e-6


def _clip_edge_to_planes(
    edge_p0: jax.Array,
    edge_p1: jax.Array,
    plane_pts: jax.Array,
    plane_normals: jax.Array,
) -> Tuple[jax.Array, jax.Array]:
  """Clips an edge against side planes.

  We return two clipped points, and a mask to include the new edge or not.

  Args:
    edge_p0: the first point on the edge
    edge_p1: the second point on the edge
    plane_pts: side plane points
    plane_normals: side plane normals

  Returns:
    new_ps: new edge points that are clipped against side planes
    mask: a boolean mask, True if an edge point is a valid clipped point and
    False otherwise
  """
  p0, p1 = edge_p0, edge_p1
  p0_in_front = jax.vmap(jp.dot)(p0 - plane_pts, plane_normals) > 1e-6
  p1_in_front = jax.vmap(jp.dot)(p1 - plane_pts, plane_normals) > 1e-6

  # Get candidate clipped points along line segment (p0, p1) by clipping against
  # all clipping planes.
  candidate_clipped_ps = jax.vmap(
      _closest_segment_point_plane, in_axes=[None, None, 0, 0]
  )(p0, p1, plane_pts, plane_normals)

  def clip_edge_point(p0, p1, p0_in_front, clipped_ps):
    @jax.vmap
    def choose_edge_point(in_front, clipped_p):
      return jp.where(in_front, clipped_p, p0)

    # Pick the clipped point if p0 is in front of the clipping plane. Otherwise
    # keep p0 as the edge point.
    new_edge_ps = choose_edge_point(p0_in_front, clipped_ps)

    # Pick the clipped point that is most along the edge direction.
    # This degenerates to picking the original point p0 if p0 is *not* in front
    # of any clipping planes.
    dists = jp.dot(new_edge_ps - p0, p1 - p0)
    new_edge_p = new_edge_ps[jp.argmax(dists)]
    return new_edge_p

  # Clip each edge point.
  new_p0 = clip_edge_point(p0, p1, p0_in_front, candidate_clipped_ps)
  new_p1 = clip_edge_point(p1, p0, p1_in_front, candidate_clipped_ps)
  clipped_pts = jp.array([new_p0, new_p1])

  # Keep the original points if both points are in front of any of the clipping
  # planes, rather than creating a new clipped edge. If the entire subject edge
  # is in front of any clipping plane, we need to grab an edge from the clipping
  # polygon instead.
  both_in_front = p0_in_front & p1_in_front
  mask = ~jp.any(both_in_front)
  new_ps = jp.where(mask, clipped_pts, jp.array([p0, p1]))
  # Mask out crossing clipped edge points.
  mask = jp.where((p0 - p1).dot(new_ps[0] - new_ps[1]) < 0, False, mask)
  return new_ps, jp.array([mask, mask])


def _clip(
    clipping_poly: jax.Array,
    subject_poly: jax.Array,
    clipping_normal: jax.Array,
    subject_normal: jax.Array,
) -> Tuple[jax.Array, jax.Array]:
  """Clips a subject polygon against a clipping polygon.

  A parallelized clipping algorithm for convex polygons. The result is a set of
  vertices on the clipped subject polygon in the subject polygon plane.

  Args:
    clipping_poly: the polygon that we use to clip the subject polygon against
    subject_poly: the polygon that gets clipped
    clipping_normal: normal of the clipping polygon
    subject_normal: normal of the subject polygon

  Returns:
    clipped_pts: points on the clipped polygon
    mask: True if a point is in the clipping polygon, False otherwise
  """
  # Get clipping edge points, edge planes, and edge normals.
  clipping_p0 = jp.roll(clipping_poly, 1, axis=0)
  clipping_plane_pts = clipping_p0
  clipping_p1 = clipping_poly
  clipping_plane_normals = jax.vmap(jp.cross, in_axes=[0, None])(
      clipping_p1 - clipping_p0,
      clipping_normal,
  )

  # Get subject edge points, edge planes, and edge normals.
  subject_edge_p0 = jp.roll(subject_poly, 1, axis=0)
  subject_plane_pts = subject_edge_p0
  subject_edge_p1 = subject_poly
  subject_plane_normals = jax.vmap(jp.cross, in_axes=[0, None])(
      subject_edge_p1 - subject_edge_p0,
      subject_normal,
  )

  # Clip all edges of the subject poly against clipping side planes.
  clipped_edges0, masks0 = jax.vmap(
      _clip_edge_to_planes, in_axes=[0, 0, None, None]
  )(
      subject_edge_p0,
      subject_edge_p1,
      clipping_plane_pts,
      clipping_plane_normals,
  )

  # Project the clipping poly onto the subject plane.
  clipping_p0_s = _project_poly_onto_poly_plane(
      clipping_p0, clipping_normal, subject_poly, subject_normal
  )
  clipping_p1_s = _project_poly_onto_poly_plane(
      clipping_p1, clipping_normal, subject_poly, subject_normal
  )

  # Clip all edges of the clipping poly against subject planes.
  clipped_edges1, masks1 = jax.vmap(
      _clip_edge_to_planes, in_axes=[0, 0, None, None]
  )(clipping_p0_s, clipping_p1_s, subject_plane_pts, subject_plane_normals)

  # Merge the points and reshape.
  clipped_edges = jp.concatenate([clipped_edges0, clipped_edges1])
  masks = jp.concatenate([masks0, masks1])
  clipped_points = clipped_edges.reshape((-1, 3))
  mask = masks.reshape(-1)

  return clipped_points, mask


def _create_contact_manifold(
    clipping_poly: jax.Array,
    subject_poly: jax.Array,
    clipping_norm: jax.Array,
    subject_norm: jax.Array,
    sep_axis: jax.Array,
) -> Tuple[jax.Array, jax.Array, jax.Array]:
  """Creates a contact manifold between two convex polygons.

  The polygon faces are expected to have a counter clockwise winding order so
  that clipping plane normals point away from the polygon center.

  Args:
    clipping_poly: the reference polygon to clip the contact against.
    subject_poly: the subject polygon to clip contacts onto.
    clipping_norm: the clipping polygon normal.
    subject_norm: the subject polygon normal.
    sep_axis: the separating axis

  Returns:
    tuple of dist, pos, and normal
  """
  # Clip the subject (incident) face onto the clipping (reference) face.
  # The incident points are clipped points on the subject polygon.
  poly_incident, mask = _clip(
      clipping_poly, subject_poly, clipping_norm, subject_norm
  )
  # The reference points are clipped points on the clipping polygon.
  poly_ref = _project_poly_onto_plane(
      poly_incident, clipping_poly[0], clipping_norm
  )
  behind_clipping_plane = _point_in_front_of_plane(
      clipping_poly[0], -clipping_norm, poly_incident
  )
  mask = mask & behind_clipping_plane

  # Choose four contact points.
  best = _manifold_points(poly_ref, mask, clipping_norm)
  contact_pts = jp.take(poly_ref, best, axis=0)
  mask_pts = jp.take(mask, best, axis=0)
  penetration_dir = jp.take(poly_incident, best, axis=0) - contact_pts
  penetration = penetration_dir.dot(-clipping_norm)

  dist = jp.where(mask_pts, -penetration, jp.ones_like(penetration))
  pos = contact_pts
  normal = -jp.stack([sep_axis] * 4, 0)
  return dist, pos, normal


def _sat_hull_hull(
    faces_a: jax.Array,
    faces_b: jax.Array,
    vertices_a: jax.Array,
    vertices_b: jax.Array,
    normals_a: jax.Array,
    normals_b: jax.Array,
    unique_edges_a: jax.Array,
    unique_edges_b: jax.Array,
) -> Tuple[jax.Array, jax.Array, jax.Array]:
  """Runs the Separating Axis Test for a pair of hulls.

  Given two convex hulls, the Separating Axis Test finds a separating axis
  between all edge pairs and face pairs. Edge pairs create a single contact
  point and face pairs create a contact manifold (up to four contact points).
  We return both the edge and face contacts. Valid contacts can be checked with
  dist < 0. Resulting edge contacts should be preferred over face contacts.

  Args:
    faces_a: An ndarray of hull A's polygon faces.
    faces_b: An ndarray of hull B's polygon faces.
    vertices_a: Vertices for hull A.
    vertices_b: Vertices for hull B.
    normals_a: Normal vectors for hull A's polygon faces.
    normals_b: Normal vectors for hull B's polygon faces.
    unique_edges_a: Unique edges for hull A.
    unique_edges_b: Unique edges for hull B.

  Returns:
    tuple of dist, pos, and normal
  """
  # get the separating axes
  edge_dir_a = unique_edges_a[:, 0] - unique_edges_a[:, 1]
  edge_dir_b = unique_edges_b[:, 0] - unique_edges_b[:, 1]
  edge_dir_a_r = jp.tile(edge_dir_a, reps=(unique_edges_b.shape[0], 1))
  edge_dir_b_r = jp.repeat(edge_dir_b, repeats=unique_edges_a.shape[0], axis=0)
  edge_edge_axes = jax.vmap(jp.cross)(edge_dir_a_r, edge_dir_b_r)
  edge_edge_axes = jax.vmap(lambda x: math.normalize(x, axis=0))(
      edge_edge_axes
  )

  axes = jp.concatenate([normals_a, normals_b, edge_edge_axes])

  # for each separating axis, get the support
  @jax.vmap
  def get_support(axis):
    support_a = jax.vmap(jp.dot, in_axes=[None, 0])(axis, vertices_a)
    support_b = jax.vmap(jp.dot, in_axes=[None, 0])(axis, vertices_b)
    dist1 = support_a.max() - support_b.min()
    dist2 = support_b.max() - support_a.min()
    sign = jp.where(dist1 > dist2, -1, 1)
    dist = jp.minimum(dist1, dist2)
    dist = jp.where(~jp.all(axis == 0.0), dist, 1e6)  # degenerate axis
    return dist, sign

  support, sign = get_support(axes)

  # choose the best separating axis
  best_idx = jp.argmin(support)
  best_sign = sign[best_idx]
  best_axis = axes[best_idx]
  is_edge_contact = best_idx >= (normals_a.shape[0] + normals_b.shape[0])

  # get the (reference) face most aligned with the separating axis
  dist_a = jax.vmap(jp.dot, in_axes=[None, 0])(best_axis, normals_a)
  dist_b = jax.vmap(jp.dot, in_axes=[None, 0])(best_axis, normals_b)
  a_max = dist_a.argmax()
  b_max = dist_b.argmax()
  a_min = dist_a.argmin()
  b_min = dist_b.argmin()

  ref_face = jp.where(best_sign > 0, faces_a[a_max], faces_b[b_max])
  ref_face_norm = jp.where(best_sign > 0, normals_a[a_max], normals_b[b_max])
  incident_face = jp.where(best_sign > 0, faces_b[b_min], faces_a[a_min])
  incident_face_norm = jp.where(
      best_sign > 0, normals_b[b_min], normals_a[a_min]
  )

  dist, pos, normal = _create_contact_manifold(
      ref_face,
      incident_face,
      ref_face_norm,
      incident_face_norm,
      -best_sign * best_axis,
  )

  # For edge contacts, we use the clipped face point, mainly for performance
  # reasons. For small penetration, the clipped face point is roughly the edge
  # contact point.
  idx = dist.argmin()
  dist = jp.where(
      is_edge_contact,
      jp.array([dist[idx], 1, 1, 1]),
      dist,
  )
  pos = jp.where(is_edge_contact, jp.tile(pos[idx], (4, 1)), pos)

  return dist, pos, normal


def plane_convex(plane: GeomInfo, convex: GeomInfo) -> Contact:
  """Calculates contacts between a plane and a convex object."""
  vert = convex.vert

  # get points in the convex frame
  plane_pos = convex.mat.T @ (plane.pos - convex.pos)
  n = convex.mat.T @ plane.mat[:, 2]
  support = (plane_pos - vert) @ n
  idx = _manifold_points(vert, support > 0, n)
  pos = vert[idx]

  # convert to world frame
  pos = convex.pos + pos @ convex.mat.T
  n = plane.mat[:, 2]

  frame = jp.stack([math.make_frame(n)] * 4, axis=0)
  unique = jp.tril(idx == idx[:, None]).sum(axis=1) == 1
  dist = jp.where(unique, -support[idx], 1)
  return dist, pos, frame


def sphere_convex(sphere: GeomInfo, convex: GeomInfo) -> Contact:
  """Calculates contact between a sphere and a convex object."""
  faces = jp.take(convex.vert, convex.face, axis=0)
  normals = convex.facenorm

  # Put sphere in convex frame.
  sphere_pos = convex.mat.T @ (sphere.pos - convex.pos)

  # Get support from face normals.
  @jax.vmap
  def get_support(faces, normal):
    pos = sphere_pos - normal * sphere.size[0]
    return jp.dot(pos - faces[0], normal)

  support = get_support(faces, normals)

  # Pick the face with minimal penetration as long as it has support.
  support = jp.where(support >= 0, -1e12, support)
  best_idx = support.argmax()
  face = faces[best_idx]
  normal = normals[best_idx]

  # Get closest point between the polygon face and the sphere center point.
  # Project the sphere center point onto poly plane. If it's inside polygon
  # edge normals, then we're done.
  pt = _project_pt_onto_plane(sphere_pos, face[0], normal)
  edge_p0 = jp.roll(face, 1, axis=0)
  edge_p1 = face
  edge_normals = jax.vmap(jp.cross, in_axes=[0, None])(
      edge_p1 - edge_p0,
      normal,
  )
  edge_dist = jax.vmap(
      lambda plane_pt, plane_norm: (pt - plane_pt).dot(plane_norm)
  )(edge_p0, edge_normals)
  inside = jp.all(edge_dist <= 0)  # lte to handle degenerate edges

  # If the point is outside edge normals, project onto the closest edge plane
  # that the point is in front of.
  degenerate_edge = jp.all(edge_normals == 0, axis=1)
  behind = edge_dist < 0.0
  edge_dist = jp.where(degenerate_edge | behind, 1e12, edge_dist)
  idx = edge_dist.argmin()
  edge_pt = math.closest_segment_point(edge_p0[idx], edge_p1[idx], pt)

  pt = jp.where(inside, pt, edge_pt)

  # Get the normal, dist, and contact position.
  n, d = math.normalize_with_norm(pt - sphere_pos)
  spt = sphere_pos + n * sphere.size[0]
  dist = d - sphere.size[0]
  pos = (pt + spt) * 0.5

  # Go back to world frame.
  n = convex.mat @ n
  pos = convex.mat @ pos + convex.pos

  return jax.tree_map(
      lambda x: jp.expand_dims(x, axis=0), (dist, pos, math.make_frame(n))
  )


def capsule_convex(cap: GeomInfo, convex: GeomInfo) -> Contact:
  """Calculates contacts between a capsule and a convex object."""
  # Get convex transformed normals, faces, and vertices.
  faces = jp.take(convex.vert, convex.face, axis=0)
  normals = convex.facenorm

  # Put capsule in convex frame.
  cap_pos = convex.mat.T @ (cap.pos - convex.pos)
  axis, length = cap.mat[:, 2], cap.size[1]
  axis = convex.mat.T @ axis
  seg = axis * length
  cap_pts = jp.array([
      cap_pos - seg,
      cap_pos + seg,
  ])

  # Get support from face normals.
  @jax.vmap
  def get_support(face, normal):
    pts = cap_pts - normal * cap.size[0]
    sup = jax.vmap(lambda x: jp.dot(x - face[0], normal))(pts)
    return sup.min()

  support = get_support(faces, normals)
  has_support = jp.all(support < 0)

  # Pick the face with minimal penetration as long as it has support.
  support = jp.where(support >= 0, -1e12, support)
  best_idx = support.argmax()
  face = faces[best_idx]
  normal = normals[best_idx]

  # Clip the edge against side planes and create two contact points against the
  # face.
  edge_p0 = jp.roll(face, 1, axis=0)
  edge_p1 = face
  edge_normals = jax.vmap(jp.cross, in_axes=[0, None])(
      edge_p1 - edge_p0,
      normal,
  )
  cap_pts_clipped, mask = _clip_edge_to_planes(
      cap_pts[0], cap_pts[1], edge_p0, edge_normals
  )
  cap_pts_clipped = cap_pts_clipped - normal * cap.size[0]
  face_pts = jax.vmap(_project_pt_onto_plane, in_axes=[0, None, None])(
      cap_pts_clipped, face[0], normal
  )
  # Create variables for the face contact.
  pos = (cap_pts_clipped + face_pts) * 0.5
  norm = jp.stack([normal] * 2, 0)
  penetration = jp.where(
      mask & has_support, jp.dot(face_pts - cap_pts_clipped, normal), -1
  )

  # Get a potential edge contact.
  edge_closest, cap_closest = jax.vmap(
      math.closest_segment_to_segment_points, in_axes=[0, 0, None, None]
  )(edge_p0, edge_p1, cap_pts[0], cap_pts[1])
  e_idx = ((edge_closest - cap_closest) ** 2).sum(axis=1).argmin()
  cap_closest_pt, edge_closest_pt = cap_closest[e_idx], edge_closest[e_idx]
  edge_axis = cap_closest_pt - edge_closest_pt
  edge_axis, edge_dist = math.normalize_with_norm(edge_axis)
  edge_pos = (
      edge_closest_pt + (cap_closest_pt - edge_axis * cap.size[0])
  ) * 0.5
  edge_norm = edge_axis
  edge_penetration = cap.size[0] - edge_dist
  has_edge_contact = edge_penetration > 0

  # Get the contact info.
  pos = jp.where(has_edge_contact, pos.at[0].set(edge_pos), pos)
  n = -jp.where(has_edge_contact, norm.at[0].set(edge_norm), norm)

  # Go back to world frame.
  pos = convex.pos + pos @ convex.mat.T
  n = n @ convex.mat.T

  dist = -jp.where(
      has_edge_contact, penetration.at[0].set(edge_penetration), penetration
  )
  frame = jax.vmap(math.make_frame)(n)
  return dist, pos, frame


def convex_convex(c1: GeomInfo, c2: GeomInfo) -> Contact:
  """Calculates contacts between two convex objects."""
  if c1.face is None or c2.face is None or c1.vert is None or c2.vert is None:
    raise AssertionError('Mesh info missing.')
  # pad face vertices so that we can broadcast between geom1 and geom2
  s1, s2 = c1.face.shape[-1], c2.face.shape[-1]
  if s1 < s2:
    face = jp.pad(c1.face, ((0, 0), (0, s2 - s1)), 'edge')
    c1 = c1.replace(face=face)
  elif s2 < s1:
    face = jp.pad(c2.face, ((0, 0), (0, s1 - s2)), 'edge')
    c2 = c2.replace(face=face)

  # ensure that the first object has fewer verts
  swapped = c1.vert.shape[0] > c2.vert.shape[0]
  if swapped:
    c1, c2 = c2, c1

  faces1 = jp.take(c1.vert, c1.face, axis=0)
  faces2 = jp.take(c2.vert, c2.face, axis=0)

  to_local_pos = c2.mat.T @ (c1.pos - c2.pos)
  to_local_mat = c2.mat.T @ c1.mat

  faces1 = to_local_pos + faces1 @ to_local_mat.T
  normals1 = c1.facenorm @ to_local_mat.T
  normals2 = c2.facenorm

  vertices1 = to_local_pos + c1.vert @ to_local_mat.T
  vertices2 = c2.vert

  unique_edges1 = jp.take(vertices1, c1.edge, axis=0)
  unique_edges2 = jp.take(vertices2, c2.edge, axis=0)

  dist, pos, normal = _sat_hull_hull(
      faces1,
      faces2,
      vertices1,
      vertices2,
      normals1,
      normals2,
      unique_edges1,
      unique_edges2,
  )

  # Go back to world frame.
  pos = c2.pos + pos @ c2.mat.T
  normal = normal @ c2.mat.T
  normal = -normal if swapped else normal

  frame = jax.vmap(math.make_frame)(normal)
  return dist, pos, frame


# store ncon as function attributes
plane_convex.ncon = 4
sphere_convex.ncon = 1
capsule_convex.ncon = 2
convex_convex.ncon = 4
