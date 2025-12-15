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

from typing import Any, Tuple

import warp as wp

MJ_MINVAL = 1e-15


wp.set_module_options({"enable_backward": False})


@wp.func
def safe_div(x: Any, y: Any) -> Any:
  return x / wp.where(y != 0.0, y, MJ_MINVAL)


@wp.func
def normalize_with_norm(x: Any):
  norm = wp.length(x)
  if norm == 0.0:
    return x, 0.0
  return x / norm, norm


@wp.func
def closest_segment_point(a: wp.vec3, b: wp.vec3, pt: wp.vec3) -> wp.vec3:
  """Returns the closest point on the a-b line segment to a point pt."""
  ab = b - a
  t = wp.dot(pt - a, ab) / (wp.dot(ab, ab) + 1e-6)
  return a + wp.clamp(t, 0.0, 1.0) * ab


@wp.func
def closest_segment_point_and_dist(a: wp.vec3, b: wp.vec3, pt: wp.vec3) -> Tuple[wp.vec3, float]:
  """Returns closest point on the line segment and the distance squared."""
  closest = closest_segment_point(a, b, pt)
  dist = wp.dot((pt - closest), (pt - closest))
  return closest, dist


@wp.func
def closest_segment_to_segment_points(a0: wp.vec3, a1: wp.vec3, b0: wp.vec3, b1: wp.vec3) -> Tuple[wp.vec3, wp.vec3]:
  """Returns closest points between two line segments."""
  dir_a, len_a = normalize_with_norm(a1 - a0)
  dir_b, len_b = normalize_with_norm(b1 - b0)

  half_len_a = len_a * 0.5
  half_len_b = len_b * 0.5
  a_mid = a0 + dir_a * half_len_a
  b_mid = b0 + dir_b * half_len_b

  trans = a_mid - b_mid

  dira_dot_dirb = wp.dot(dir_a, dir_b)
  dira_dot_trans = wp.dot(dir_a, trans)
  dirb_dot_trans = wp.dot(dir_b, trans)
  denom = 1.0 - dira_dot_dirb * dira_dot_dirb

  orig_t_a = (-dira_dot_trans + dira_dot_dirb * dirb_dot_trans) / (denom + 1e-6)
  orig_t_b = dirb_dot_trans + orig_t_a * dira_dot_dirb
  t_a = wp.clamp(orig_t_a, -half_len_a, half_len_a)
  t_b = wp.clamp(orig_t_b, -half_len_b, half_len_b)

  best_a = a_mid + dir_a * t_a
  best_b = b_mid + dir_b * t_b

  new_a, d1 = closest_segment_point_and_dist(a0, a1, best_b)
  new_b, d2 = closest_segment_point_and_dist(b0, b1, best_a)
  if d1 < d2:
    return new_a, best_b
  return best_a, new_b


class vec8f(wp.types.vector(length=8, dtype=wp.float32)):
  pass


class mat23f(wp.types.matrix(shape=(2, 3), dtype=wp.float32)):
  pass


class mat43f(wp.types.matrix(shape=(4, 3), dtype=wp.float32)):
  pass


class mat83f(wp.types.matrix(shape=(8, 3), dtype=wp.float32)):
  pass


# core
@wp.func
def plane_sphere(plane_normal: wp.vec3, plane_pos: wp.vec3, sphere_pos: wp.vec3, sphere_radius: float) -> Tuple[float, wp.vec3]:
  # TODO(team): docstring
  dist = wp.dot(sphere_pos - plane_pos, plane_normal) - sphere_radius
  pos = sphere_pos - plane_normal * (sphere_radius + 0.5 * dist)
  return dist, pos


@wp.func
def sphere_sphere(
  # In:
  pos1: wp.vec3,
  radius1: float,
  pos2: wp.vec3,
  radius2: float,
) -> Tuple[float, wp.vec3, wp.vec3]:
  """Sphere-sphere collision calculation.

  Args:
    pos1: Center position of the first sphere.
    radius1: Radius of the first sphere.
    pos2: Center position of the second sphere.
    radius2: Radius of the second sphere.

  Returns:
    - Distance between sphere surfaces (negative if overlapping).
    - Contact position.
    - Contact normal vector.
  """
  dir = pos2 - pos1
  dist = wp.length(dir)
  if dist == 0.0:
    n = wp.vec3(1.0, 0.0, 0.0)
  else:
    n = dir / dist
  dist = dist - (radius1 + radius2)
  pos = pos1 + n * (radius1 + 0.5 * dist)
  return dist, pos, n


@wp.func
def sphere_capsule(
  # In:
  sphere_pos: wp.vec3,
  sphere_radius: float,
  capsule_pos: wp.vec3,
  capsule_axis: wp.vec3,
  capsule_radius: float,
  capsule_half_length: float,
) -> Tuple[float, wp.vec3, wp.vec3]:
  """Core contact geometry calculation for sphere-capsule collision.

  Args:
    sphere_pos: Center position of the sphere.
    sphere_radius: Radius of the sphere.
    capsule_pos: Center position of the capsule.
    capsule_axis: Axis direction of the capsule.
    capsule_radius: Radius of the capsule.
    capsule_half_length: Half length of the capsule.

  Returns:
    - Vector of contact distances.
    - Matrix of contact positions (one per row).
    - Matrix of contact normal vectors (one per row).
  """
  # Calculate capsule segment
  segment = capsule_axis * capsule_half_length

  # Find closest point on capsule centerline to sphere center
  pt = closest_segment_point(capsule_pos - segment, capsule_pos + segment, sphere_pos)

  # Use sphere-sphere collision between sphere and closest point
  return sphere_sphere(sphere_pos, sphere_radius, pt, capsule_radius)


@wp.func
def capsule_capsule(
  # In:
  cap1_pos: wp.vec3,
  cap1_axis: wp.vec3,
  cap1_radius: float,
  cap1_half_length: float,
  cap2_pos: wp.vec3,
  cap2_axis: wp.vec3,
  cap2_radius: float,
  cap2_half_length: float,
) -> Tuple[float, wp.vec3, wp.vec3]:
  """Core contact geometry calculation for capsule-capsule collision.

  Args:
    cap1_pos: Center position of the first capsule.
    cap1_axis: Axis direction of the first capsule.
    cap1_radius: Radius of the first capsule.
    cap1_half_length: Half length of the first capsule.
    cap2_pos: Center position of the second capsule.
    cap2_axis: Axis direction of the second capsule.
    cap2_radius: Radius of the second capsule.
    cap2_half_length: Half length of the second capsule.

  Returns:
    - Vector of contact distances.
    - Matrix of contact positions (one per row).
    - Matrix of contact normal vectors (one per row).
  """
  # TODO(team): parallel axes case

  # Calculate capsule segments
  seg1 = cap1_axis * cap1_half_length
  seg2 = cap2_axis * cap2_half_length

  # Find closest points between capsule centerlines
  pt1, pt2 = closest_segment_to_segment_points(
    cap1_pos - seg1,
    cap1_pos + seg1,
    cap2_pos - seg2,
    cap2_pos + seg2,
  )

  # Use sphere-sphere collision between closest points
  return sphere_sphere(pt1, cap1_radius, pt2, cap2_radius)


@wp.func
def plane_capsule(
  # In:
  plane_normal: wp.vec3,
  plane_pos: wp.vec3,
  capsule_pos: wp.vec3,
  capsule_axis: wp.vec3,
  capsule_radius: float,
  capsule_half_length: float,
) -> Tuple[wp.vec2, mat23f, wp.mat33]:
  """Core contact geometry calculation for plane-capsule collision.

  Args:
    plane_normal: Normal vector of the plane.
    plane_pos: Position point on the plane.
    capsule_pos: Center position of the capsule.
    capsule_axis: Axis direction of the capsule.
    capsule_radius: Radius of the capsule.
    capsule_half_length: Half length of the capsule.

  Returns:
    - Vector of contact distances.
    - Matrix of contact positions (one per row).
    - Contact frame for both contacts.
  """
  n = plane_normal
  axis = capsule_axis

  # align contact frames with capsule axis
  b, b_norm = normalize_with_norm(axis - n * wp.dot(n, axis))

  if b_norm < 0.5:
    if -0.5 < n[1] and n[1] < 0.5:
      b = wp.vec3(0.0, 1.0, 0.0)
    else:
      b = wp.vec3(0.0, 0.0, 1.0)

  c = wp.cross(n, b)
  frame = wp.mat33(n[0], n[1], n[2], b[0], b[1], b[2], c[0], c[1], c[2])
  segment = axis * capsule_half_length

  # First contact (positive end of capsule)
  dist1, pos1 = plane_sphere(n, plane_pos, capsule_pos + segment, capsule_radius)

  # Second contact (negative end of capsule)
  dist2, pos2 = plane_sphere(n, plane_pos, capsule_pos - segment, capsule_radius)

  dist = wp.vec2(dist1, dist2)
  pos = mat23f(pos1[0], pos1[1], pos1[2], pos2[0], pos2[1], pos2[2])

  return dist, pos, frame


@wp.func
def plane_ellipsoid(
  # In:
  plane_normal: wp.vec3,
  plane_pos: wp.vec3,
  ellipsoid_pos: wp.vec3,
  ellipsoid_rot: wp.mat33,
  ellipsoid_size: wp.vec3,
) -> Tuple[float, wp.vec3, wp.vec3]:
  """Core contact geometry calculation for plane-ellipsoid collision.

  Args:
    plane_normal: Normal vector of the plane.
    plane_pos: Position point on the plane.
    ellipsoid_pos: Center position of the ellipsoid.
    ellipsoid_rot: Rotation matrix of the ellipsoid.
    ellipsoid_size: Size (radii) of the ellipsoid along each axis.

  Returns:
    - Vector of contact distances.
    - Matrix of contact positions (one per row).
    - Matrix of contact normal vectors (one per row).
  """
  sphere_support = -wp.normalize(wp.cw_mul(wp.transpose(ellipsoid_rot) @ plane_normal, ellipsoid_size))
  pos = ellipsoid_pos + ellipsoid_rot @ wp.cw_mul(sphere_support, ellipsoid_size)
  dist = wp.dot(plane_normal, pos - plane_pos)
  pos = pos - plane_normal * dist * 0.5

  return dist, pos, plane_normal


@wp.func
def plane_box(
  # In:
  plane_normal: wp.vec3,
  plane_pos: wp.vec3,
  box_pos: wp.vec3,
  box_rot: wp.mat33,
  box_size: wp.vec3,
) -> Tuple[vec8f, mat83f, wp.vec3]:
  """Core contact geometry calculation for plane-box collision.

  Args:
    plane_normal: Normal vector of the plane.
    plane_pos: Position point on the plane.
    box_pos: Center position of the box.
    box_rot: Rotation matrix of the box.
    box_size: Half-extents of the box along each axis.
    margin: Collision tolerance.

  Returns:
    - Vector of contact distances (wp.inf for unpopulated contacts).
    - Matrix of contact positions (one per row).
    - Contact normal vector.
  """
  center_dist = wp.dot(box_pos - plane_pos, plane_normal)

  dist = vec8f(wp.inf)
  pos = mat83f()

  # test all corners, pick bottom 4
  for i in range(8):
    # get corner in local coordinates
    corner = wp.vec3(
      wp.where(i & 1, box_size[0], -box_size[0]),
      wp.where(i & 2, box_size[1], -box_size[1]),
      wp.where(i & 4, box_size[2], -box_size[2]),
    )

    # get corner in global coordinates relative to box center
    corner = box_rot * corner

    # compute distance to plane
    ldist = wp.dot(plane_normal, corner)
    cdist = center_dist + ldist

    dist[i] = cdist
    pos[i] = corner + box_pos - 0.5 * plane_normal * cdist

  return dist, pos, plane_normal


@wp.func
def sphere_cylinder(
  # In:
  sphere_pos: wp.vec3,
  sphere_radius: float,
  cylinder_pos: wp.vec3,
  cylinder_axis: wp.vec3,
  cylinder_radius: float,
  cylinder_half_height: float,
) -> Tuple[float, wp.vec3, wp.vec3]:
  """Core contact geometry calculation for sphere-cylinder collision.

  Args:
    sphere_pos: Center position of the sphere.
    sphere_radius: Radius of the sphere.
    cylinder_pos: Center position of the cylinder.
    cylinder_axis: Axis direction of the cylinder.
    cylinder_radius: Radius of the cylinder.
    cylinder_half_height: Half height of the cylinder.

  Returns:
    - Vector of contact distances.
    - Matrix of contact positions (one per row).
    - Matrix of contact normal vectors (one per row).
  """
  vec = sphere_pos - cylinder_pos
  x = wp.dot(vec, cylinder_axis)

  a_proj = cylinder_axis * x
  p_proj = vec - a_proj
  p_proj_sqr = wp.dot(p_proj, p_proj)

  collide_side = wp.abs(x) < cylinder_half_height
  collide_cap = p_proj_sqr < (cylinder_radius * cylinder_radius)

  if collide_side and collide_cap:
    dist_cap = cylinder_half_height - wp.abs(x)
    dist_radius = cylinder_radius - wp.sqrt(p_proj_sqr)

    if dist_cap < dist_radius:
      collide_side = False
    else:
      collide_cap = False

  # side collision
  if collide_side:
    pos_target = cylinder_pos + a_proj
    return sphere_sphere(sphere_pos, sphere_radius, pos_target, cylinder_radius)
  # cap collision
  elif collide_cap:
    if x > 0.0:
      # top cap
      pos_cap = cylinder_pos + cylinder_axis * cylinder_half_height
      plane_normal = cylinder_axis
    else:
      # bottom cap
      pos_cap = cylinder_pos - cylinder_axis * cylinder_half_height
      plane_normal = -cylinder_axis

    dist, pos = plane_sphere(plane_normal, pos_cap, sphere_pos, sphere_radius)
    return dist, pos, -plane_normal  # flip normal after position calculation
  # corner collision
  else:
    inv_len = safe_div(1.0, wp.sqrt(p_proj_sqr))
    p_proj = p_proj * (cylinder_radius * inv_len)

    cap_offset = cylinder_axis * (wp.sign(x) * cylinder_half_height)
    pos_corner = cylinder_pos + cap_offset + p_proj

    return sphere_sphere(sphere_pos, sphere_radius, pos_corner, 0.0)


@wp.func
def plane_cylinder(
  # In:
  plane_normal: wp.vec3,
  plane_pos: wp.vec3,
  cylinder_center: wp.vec3,
  cylinder_axis: wp.vec3,
  cylinder_radius: float,
  cylinder_half_height: float,
) -> Tuple[wp.vec4, mat43f, wp.vec3]:
  """Core contact geometry calculation for plane-cylinder collision.

  Args:
    plane_normal: Normal vector of the plane.
    plane_pos: Position point on the plane.
    cylinder_center: Center position of the cylinder.
    cylinder_axis: Axis direction of the cylinder.
    cylinder_radius: Radius of the cylinder.
    cylinder_half_height: Half height of the cylinder.

  Returns:
    - Vector of contact distances.
    - Matrix of contact positions (one per row).
    - Matrix of contact normal vectors (one per row).
  """
  # Initialize output matrices
  contact_dist = wp.vec4(wp.inf)
  contact_pos = mat43f()
  contact_count = 0

  n = plane_normal
  axis = cylinder_axis

  # Project, make sure axis points toward plane
  prjaxis = wp.dot(n, axis)
  if prjaxis > 0:
    axis = -axis
    prjaxis = -prjaxis

  # Compute normal distance from plane to cylinder center
  dist0 = wp.dot(cylinder_center - plane_pos, n)

  # Remove component of -normal along cylinder axis
  vec = axis * prjaxis - n
  len_sqr = wp.dot(vec, vec)

  # If vector is nondegenerate, normalize and scale by radius
  # Otherwise use cylinder's x-axis scaled by radius
  vec = wp.where(
    len_sqr >= 1e-12,
    vec * safe_div(cylinder_radius, wp.sqrt(len_sqr)),
    wp.vec3(1.0, 0.0, 0.0) * cylinder_radius,  # Default x-axis when degenerate
  )

  # Project scaled vector on normal
  prjvec = wp.dot(vec, n)

  # Scale cylinder axis by half-length
  axis = axis * cylinder_half_height
  prjaxis = prjaxis * cylinder_half_height

  # First contact point (end cap closer to plane)
  dist1 = dist0 + prjaxis + prjvec
  pos1 = cylinder_center + vec + axis - n * (dist1 * 0.5)
  contact_dist[contact_count] = dist1
  contact_pos[contact_count] = pos1
  contact_count = contact_count + 1

  # Second contact point (end cap farther from plane)
  dist2 = dist0 - prjaxis + prjvec
  pos2 = cylinder_center + vec - axis - n * (dist2 * 0.5)
  contact_dist[contact_count] = dist2
  contact_pos[contact_count] = pos2
  contact_count = contact_count + 1

  # Try triangle contact points on side closer to plane
  prjvec1 = -prjvec * 0.5
  dist3 = dist0 + prjaxis + prjvec1
  # Compute sideways vector scaled by radius*sqrt(3)/2
  vec1 = wp.cross(vec, axis)
  vec1 = wp.normalize(vec1) * (cylinder_radius * wp.sqrt(3.0) * 0.5)

  # Add contact point A - adjust to closest side
  pos3 = cylinder_center + vec1 + axis - vec * 0.5 - n * (dist3 * 0.5)
  contact_dist[contact_count] = dist3
  contact_pos[contact_count] = pos3
  contact_count = contact_count + 1

  # Add contact point B - adjust to closest side
  pos4 = cylinder_center - vec1 + axis - vec * 0.5 - n * (dist3 * 0.5)
  contact_dist[contact_count] = dist3
  contact_pos[contact_count] = pos4
  contact_count = contact_count + 1

  return contact_dist, contact_pos, n


@wp.func
def _compute_rotmore(face_idx: int) -> wp.mat33:
  rotmore = wp.mat33(0.0)

  if face_idx == 0:
    rotmore[0, 2] = -1.0
    rotmore[1, 1] = +1.0
    rotmore[2, 0] = +1.0
  elif face_idx == 1:
    rotmore[0, 0] = +1.0
    rotmore[1, 2] = -1.0
    rotmore[2, 1] = +1.0
  elif face_idx == 2:
    rotmore[0, 0] = +1.0
    rotmore[1, 1] = +1.0
    rotmore[2, 2] = +1.0
  elif face_idx == 3:
    rotmore[0, 2] = +1.0
    rotmore[1, 1] = +1.0
    rotmore[2, 0] = -1.0
  elif face_idx == 4:
    rotmore[0, 0] = +1.0
    rotmore[1, 2] = +1.0
    rotmore[2, 1] = -1.0
  elif face_idx == 5:
    rotmore[0, 0] = -1.0
    rotmore[1, 1] = +1.0
    rotmore[2, 2] = -1.0

  return rotmore


@wp.func
def box_box(
  # In:
  box1_pos: wp.vec3,
  box1_rot: wp.mat33,
  box1_size: wp.vec3,
  box2_pos: wp.vec3,
  box2_rot: wp.mat33,
  box2_size: wp.vec3,
  margin: float = 0.0,  # kernel_analyzer: off
) -> Tuple[vec8f, mat83f, mat83f]:
  """Core contact geometry calculation for box-box collision.

  Args:
    box1_pos: Center position of the first box.
    box1_rot: Rotation matrix of the first box.
    box1_size: Half-extents of the first box along each axis.
    box2_pos: Center position of the second box.
    box2_rot: Rotation matrix of the second box.
    box2_size: Half-extents of the second box along each axis.
    margin: Collision tolerance.

  Returns:
    - Vector of contact distances (wp.inf for unpopulated contacts).
    - Matrix of contact positions (one per row).
    - Matrix of contact normal vectors (one per row).
  """
  # Initialize output matrices
  contact_dist = vec8f()
  for i in range(8):
    contact_dist[i] = wp.inf
  contact_pos = mat83f()
  contact_normals = mat83f()
  contact_count = 0

  # Compute transforms between box's frames
  pos21 = wp.transpose(box1_rot) @ (box2_pos - box1_pos)
  pos12 = wp.transpose(box2_rot) @ (box1_pos - box2_pos)

  rot21 = wp.transpose(box1_rot) @ box2_rot
  rot12 = wp.transpose(rot21)

  rot21abs = wp.matrix_from_rows(wp.abs(rot21[0]), wp.abs(rot21[1]), wp.abs(rot21[2]))
  rot12abs = wp.transpose(rot21abs)

  plen2 = rot21abs @ box2_size
  plen1 = rot12abs @ box1_size

  # Compute axis of maximum separation
  s_sum_3 = 3.0 * (box1_size + box2_size)
  separation = wp.float32(margin + s_sum_3[0] + s_sum_3[1] + s_sum_3[2])
  axis_code = wp.int32(-1)

  # First test: consider boxes' face normals
  for i in range(3):
    c1 = -wp.abs(pos21[i]) + box1_size[i] + plen2[i]

    c2 = -wp.abs(pos12[i]) + box2_size[i] + plen1[i]

    if c1 < -margin or c2 < -margin:
      return contact_dist, contact_pos, contact_normals

    if c1 < separation:
      separation = c1
      axis_code = i + 3 * wp.int32(pos21[i] < 0) + 0  # Face of box1
    if c2 < separation:
      separation = c2
      axis_code = i + 3 * wp.int32(pos12[i] < 0) + 6  # Face of box2

  clnorm = wp.vec3(0.0)
  inv = wp.bool(False)
  cle1 = wp.int32(0)
  cle2 = wp.int32(0)

  # Second test: consider cross products of boxes' edges
  for i in range(3):
    for j in range(3):
      # Compute cross product of box edges (potential separating axis)
      if i == 0:
        cross_axis = wp.vec3(0.0, -rot12[j, 2], rot12[j, 1])
      elif i == 1:
        cross_axis = wp.vec3(rot12[j, 2], 0.0, -rot12[j, 0])
      else:
        cross_axis = wp.vec3(-rot12[j, 1], rot12[j, 0], 0.0)

      cross_length = wp.length(cross_axis)
      if cross_length < MJ_MINVAL:
        continue

      cross_axis /= cross_length

      box_dist = wp.dot(pos21, cross_axis)
      c3 = wp.float32(0.0)

      # Project box half-sizes onto the potential separating axis
      for k in range(3):
        if k != i:
          c3 += box1_size[k] * wp.abs(cross_axis[k])
        if k != j:
          c3 += box2_size[k] * rot21abs[i, 3 - k - j] / cross_length

      c3 -= wp.abs(box_dist)

      # Early exit: no collision if separated along this axis
      if c3 < -margin:
        return contact_dist, contact_pos, contact_normals

      # Track minimum separation and which edge-edge pair it occurs on
      if c3 < separation * (1.0 - 1e-12):
        separation = c3
        # Determine which corners/edges are closest
        cle1 = 0
        cle2 = 0

        for k in range(3):
          if k != i and (int(cross_axis[k] > 0) ^ int(box_dist < 0)):
            cle1 += 1 << k
          if k != j:
            if int(rot21[i, 3 - k - j] > 0) ^ int(box_dist < 0) ^ int((k - j + 3) % 3 == 1):
              cle2 += 1 << k

        axis_code = 12 + i * 3 + j
        clnorm = cross_axis
        inv = box_dist < 0

  # No axis with separation < margin found
  if axis_code == -1:
    return contact_dist, contact_pos, contact_normals

  points = mat83f()
  depth = vec8f()
  max_con_pair = 8
  # 8 contacts should suffice for most configurations

  if axis_code < 12:
    # Handle face-vertex collision
    face_idx = axis_code % 6
    box_idx = axis_code // 6
    rotmore = _compute_rotmore(face_idx)

    r = rotmore @ wp.where(box_idx, rot12, rot21)
    p = rotmore @ wp.where(box_idx, pos12, pos21)
    ss = wp.abs(rotmore @ wp.where(box_idx, box2_size, box1_size))
    s = wp.where(box_idx, box1_size, box2_size)
    rt = wp.transpose(r)

    lx, ly, hz = ss[0], ss[1], ss[2]
    p[2] -= hz

    clcorner = wp.int32(0)  # corner of non-face box with least axis separation

    for i in range(3):
      if r[2, i] < 0:
        clcorner += 1 << i

    lp = p
    for i in range(wp.static(3)):
      lp += rt[i] * s[i] * wp.where(clcorner & 1 << i, 1.0, -1.0)

    m = wp.int32(1)
    dirs = wp.int32(0)

    cn1 = wp.vec3(0.0)
    cn2 = wp.vec3(0.0)

    for i in range(3):
      if wp.abs(r[2, i]) < 0.5:
        if not dirs:
          cn1 = rt[i] * s[i] * wp.where(clcorner & (1 << i), -2.0, 2.0)
        else:
          cn2 = rt[i] * s[i] * wp.where(clcorner & (1 << i), -2.0, 2.0)

        dirs += 1

    k = dirs * dirs

    # Find potential contact points

    n = wp.int32(0)

    for i in range(k):
      for q in range(2):
        # lines_a and lines_b (lines between corners) computed on the fly
        lav = lp + wp.where(i < 2, wp.vec3(0.0), wp.where(i == 2, cn1, cn2))
        lbv = wp.where(i == 0 or i == 3, cn1, cn2)

        if wp.abs(lbv[q]) > MJ_MINVAL:
          br = 1.0 / lbv[q]
          for j in range(-1, 2, 2):
            l = ss[q] * wp.float32(j)
            c1 = (l - lav[q]) * br
            if c1 < 0 or c1 > 1:
              continue
            c2 = lav[1 - q] + lbv[1 - q] * c1
            if wp.abs(c2) > ss[1 - q]:
              continue

            points[n] = lav + c1 * lbv
            n += 1

    if dirs == 2:
      ax = cn1[0]
      bx = cn2[0]
      ay = cn1[1]
      by = cn2[1]
      C = safe_div(1.0, ax * by - bx * ay)

      for i in range(4):
        llx = wp.where(i // 2, lx, -lx)
        lly = wp.where(i % 2, ly, -ly)

        x = llx - lp[0]
        y = lly - lp[1]

        u = (x * by - y * bx) * C
        v = (y * ax - x * ay) * C

        if u > 0 and v > 0 and u < 1 and v < 1:
          points[n] = wp.vec3(llx, lly, lp[2] + u * cn1[2] + v * cn2[2])
          n += 1

    for i in range(1 << dirs):
      tmpv = lp + wp.float32(i & 1) * cn1 + wp.float32((i & 2) != 0) * cn2
      if tmpv[0] > -lx and tmpv[0] < lx and tmpv[1] > -ly and tmpv[1] < ly:
        points[n] = tmpv
        n += 1

    m = n
    n = wp.int32(0)

    for i in range(m):
      if points[i][2] > margin:
        continue
      if i != n:
        points[n] = points[i]

      depth[n] = points[n, 2]
      points[n, 2] *= 0.5
      n += 1

    # Set up contact frame
    rw = wp.where(box_idx, box2_rot, box1_rot) @ wp.transpose(rotmore)
    pw = wp.where(box_idx, box2_pos, box1_pos)
    normal = wp.where(box_idx, -1.0, 1.0) * wp.transpose(rw)[2]

  else:
    # Handle edge-edge collision
    edge1 = (axis_code - 12) // 3
    edge2 = (axis_code - 12) % 3

    # Set up non-contacting edges ax1, ax2 for box2 and pax1, pax2 for box 1
    ax1 = wp.int(1 - (edge2 & 1))
    ax2 = wp.int(2 - (edge2 & 2))

    pax1 = wp.int(1 - (edge1 & 1))
    pax2 = wp.int(2 - (edge1 & 2))

    if rot21abs[edge1, ax1] < rot21abs[edge1, ax2]:
      ax1, ax2 = ax2, ax1

    if rot12abs[edge2, pax1] < rot12abs[edge2, pax2]:
      pax1, pax2 = pax2, pax1

    rotmore = _compute_rotmore(wp.where(cle1 & (1 << pax2), pax2, pax2 + 3))

    # Transform coordinates for edge-edge contact calculation
    p = rotmore @ pos21
    rnorm = rotmore @ clnorm
    r = rotmore @ rot21
    rt = wp.transpose(r)
    s = wp.abs(wp.transpose(rotmore) @ box1_size)

    lx, ly, hz = s[0], s[1], s[2]
    p[2] -= hz

    # Calculate closest box2 face

    points[0] = (
      p
      + rt[ax1] * box2_size[ax1] * wp.where(cle2 & (1 << ax1), 1.0, -1.0)
      + rt[ax2] * box2_size[ax2] * wp.where(cle2 & (1 << ax2), 1.0, -1.0)
    )
    points[1] = points[0] - rt[edge2] * box2_size[edge2]
    points[0] += rt[edge2] * box2_size[edge2]

    points[2] = (
      p
      + rt[ax1] * box2_size[ax1] * wp.where(cle2 & (1 << ax1), -1.0, 1.0)
      + rt[ax2] * box2_size[ax2] * wp.where(cle2 & (1 << ax2), 1.0, -1.0)
    )

    points[3] = points[2] - rt[edge2] * box2_size[edge2]
    points[2] += rt[edge2] * box2_size[edge2]

    n = 4

    # Set up coordinate axes for contact face of box2
    axi_lp = points[0]
    axi_cn1 = points[1] - points[0]
    axi_cn2 = points[2] - points[0]

    # Check if contact normal is valid
    if wp.abs(rnorm[2]) < MJ_MINVAL:
      return contact_dist, contact_pos, contact_normals  # Shouldn't happen

    # Calculate inverse normal for projection
    innorm = wp.where(inv, -1.0, 1.0) / rnorm[2]

    pu = mat43f()

    # Project points onto contact plane
    for i in range(4):
      pu[i] = points[i]
      c_scl = points[i, 2] * wp.where(inv, -1.0, 1.0) * innorm
      points[i] -= rnorm * c_scl

    pts_lp = points[0]
    pts_cn1 = points[1] - points[0]
    pts_cn2 = points[2] - points[0]

    n = wp.int32(0)

    for i in range(4):
      for q in range(2):
        la = pts_lp[q] + wp.where(i < 2, 0.0, wp.where(i == 2, pts_cn1[q], pts_cn2[q]))
        lb = wp.where(i == 0 or i == 3, pts_cn1[q], pts_cn2[q])
        lc = pts_lp[1 - q] + wp.where(i < 2, 0.0, wp.where(i == 2, pts_cn1[1 - q], pts_cn2[1 - q]))
        ld = wp.where(i == 0 or i == 3, pts_cn1[1 - q], pts_cn2[1 - q])

        # linesu_a and linesu_b (lines between corners) computed on the fly
        lua = axi_lp + wp.where(i < 2, wp.vec3(0.0), wp.where(i == 2, axi_cn1, axi_cn2))
        lub = wp.where(i == 0 or i == 3, axi_cn1, axi_cn2)

        if wp.abs(lb) > MJ_MINVAL:
          br = 1.0 / lb
          for j in range(-1, 2, 2):
            if n == max_con_pair:
              break
            l = s[q] * wp.float32(j)
            c1 = (l - la) * br
            if c1 < 0 or c1 > 1:
              continue
            c2 = lc + ld * c1
            if wp.abs(c2) > s[1 - q]:
              continue
            if (lua[2] + lub[2] * c1) * innorm > margin:
              continue

            points[n] = lua * 0.5 + c1 * lub * 0.5
            points[n, q] += 0.5 * l
            points[n, 1 - q] += 0.5 * c2
            depth[n] = points[n, 2] * innorm * 2.0
            n += 1

    nl = n

    ax = pts_cn1[0]
    bx = pts_cn2[0]
    ay = pts_cn1[1]
    by = pts_cn2[1]
    C = safe_div(1.0, ax * by - bx * ay)

    for i in range(4):
      if n == max_con_pair:
        break
      llx = wp.where(i // 2, lx, -lx)
      lly = wp.where(i % 2, ly, -ly)

      x = llx - pts_lp[0]
      y = lly - pts_lp[1]

      u = (x * by - y * bx) * C
      v = (y * ax - x * ay) * C

      if nl == 0:
        if (u < 0 or u > 1) and (v < 0 or v > 1):
          continue
      elif u < 0 or v < 0 or u > 1 or v > 1:
        continue

      u = wp.clamp(u, 0.0, 1.0)
      v = wp.clamp(v, 0.0, 1.0)
      w = 1.0 - u - v
      vtmp = pu[0] * w + pu[1] * u + pu[2] * v

      points[n] = wp.vec3(llx, lly, 0.0)

      vtmp2 = points[n] - vtmp
      tc1 = wp.length_sq(vtmp2)
      if vtmp[2] > 0 and tc1 > margin * margin:
        continue

      points[n] = 0.5 * (points[n] + vtmp)

      depth[n] = wp.sqrt(tc1) * wp.where(vtmp[2] < 0, -1.0, 1.0)
      n += 1

    nf = n

    for i in range(4):
      if n >= max_con_pair:
        break
      x = pu[i, 0]
      y = pu[i, 1]
      if nl == 0 and nf != 0:
        if (x < -lx or x > lx) and (y < -ly or y > ly):
          continue
      elif x < -lx or x > lx or y < -ly or y > ly:
        continue

      c1 = wp.float32(0)

      for j in range(2):
        if pu[i, j] < -s[j]:
          c1 += (pu[i, j] + s[j]) * (pu[i, j] + s[j])
        elif pu[i, j] > s[j]:
          c1 += (pu[i, j] - s[j]) * (pu[i, j] - s[j])

      c1 += pu[i, 2] * innorm * pu[i, 2] * innorm

      if pu[i, 2] > 0 and c1 > margin * margin:
        continue

      tmp_p = wp.vec3(pu[i, 0], pu[i, 1], 0.0)

      for j in range(2):
        if pu[i, j] < -s[j]:
          tmp_p[j] = -s[j] * 0.5
        elif pu[i, j] > s[j]:
          tmp_p[j] = +s[j] * 0.5

      tmp_p += pu[i]
      points[n] = tmp_p * 0.5

      depth[n] = wp.sqrt(c1) * wp.where(pu[i, 2] < 0, -1.0, 1.0)
      n += 1

    # Set up contact data for all points
    rw = box1_rot @ wp.transpose(rotmore)
    pw = box1_pos
    normal = wp.where(inv, -1.0, 1.0) * rw @ rnorm

  contact_count = n

  # Copy contact data to output matrices
  for i in range(contact_count):
    points[i, 2] += hz
    pos = rw @ points[i] + pw
    contact_dist[i] = depth[i]
    contact_pos[i] = pos
    contact_normals[i] = normal

  return contact_dist, contact_pos, contact_normals


@wp.func
def sphere_box(
  # In:
  sphere_pos: wp.vec3,
  sphere_radius: float,
  box_pos: wp.vec3,
  box_rot: wp.mat33,
  box_size: wp.vec3,
) -> Tuple[float, wp.vec3, wp.vec3]:
  """Core contact geometry calculation for sphere-box collision.

  Args:
    sphere_pos: Center position of the sphere.
    sphere_radius: Radius of the sphere.
    box_pos: Center position of the box.
    box_rot: Rotation matrix of the box.
    box_size: Half-extents of the box along each axis.

  Returns:
    - Vector of contact distances.
    - Contact positions.
    - Contact normal vectors.
  """
  center = wp.transpose(box_rot) @ (sphere_pos - box_pos)

  clamped = wp.max(-box_size, wp.min(box_size, center))
  clamped_dir, dist = normalize_with_norm(clamped - center)

  # sphere center inside box
  if dist <= MJ_MINVAL:
    closest = 2.0 * (box_size[0] + box_size[1] + box_size[2])
    k = wp.int32(0)
    for i in range(6):
      face_dist = wp.abs(wp.where(i % 2, 1.0, -1.0) * box_size[i // 2] - center[i // 2])
      if closest > face_dist:
        closest = face_dist
        k = i

    nearest = wp.vec3(0.0)
    nearest[k // 2] = wp.where(k % 2, -1.0, 1.0)
    pos = center + nearest * (sphere_radius - closest) / 2.0
    contact_normal = box_rot @ nearest
    contact_distance = -closest - sphere_radius

  else:
    deepest = center + clamped_dir * sphere_radius
    pos = 0.5 * (clamped + deepest)
    contact_normal = box_rot @ clamped_dir
    contact_distance = dist - sphere_radius

  contact_position = box_pos + box_rot @ pos

  return contact_distance, contact_position, contact_normal


@wp.func
def capsule_box(
  # In:
  capsule_pos: wp.vec3,
  capsule_axis: wp.vec3,
  capsule_radius: float,
  capsule_half_length: float,
  box_pos: wp.vec3,
  box_rot: wp.mat33,
  box_size: wp.vec3,
) -> Tuple[wp.vec2, mat23f, mat23f]:
  """Core contact geometry calculation for capsule-box collision.

  Args:
    capsule_pos: Center position of the capsule.
    capsule_axis: Axis direction of the capsule.
    capsule_radius: Radius of the capsule.
    capsule_half_length: Half length of the capsule.
    box_pos: Center position of the box.
    box_rot: Rotation matrix of the box.
    box_size: Half-extents of the box along each axis.

  Returns:
    - Vector of contact distances (wp.inf for unpopulated contacts).
    - Matrix of contact positions (one per row).
    - Matrix of contact normal vectors (one per row).
  """
  # Based on the mjc implementation
  boxmatT = wp.transpose(box_rot)
  pos = boxmatT @ (capsule_pos - box_pos)
  axis = boxmatT @ capsule_axis
  halfaxis = axis * capsule_half_length  # halfaxis is the capsule direction
  axisdir = wp.int32(halfaxis[0] > 0.0) + 2 * wp.int32(halfaxis[1] > 0.0) + 4 * wp.int32(halfaxis[2] > 0.0)

  # keep track of closest point
  bestdist = wp.float32(1.0e32)
  bestsegmentpos = wp.float32(-12)

  # cltype: encoded collision configuration
  # cltype / 3 == 0 : lower corner is closest to the capsule
  #            == 2 : upper corner is closest to the capsule
  #            == 1 : middle of the edge is closest to the capsule
  # cltype % 3 == 0 : lower corner is closest to the box
  #            == 2 : upper corner is closest to the box
  #            == 1 : middle of the capsule is closest to the box
  cltype = wp.int32(-4)

  # clface: index of the closest face of the box to the capsule
  # -1: no face is closest (edge or corner is closest)
  # 0, 1, 2: index of the axis perpendicular to the closest face
  clface = wp.int32(-12)

  # first: consider cases where a face of the box is closest
  for i in range(-1, 2, 2):
    axisTip = pos + wp.float32(i) * halfaxis
    boxPoint = wp.vec3(axisTip)

    n_out = wp.int32(0)
    ax_out = wp.int32(-1)

    for j in range(3):
      if boxPoint[j] < -box_size[j]:
        n_out += 1
        ax_out = j
        boxPoint[j] = -box_size[j]
      elif boxPoint[j] > box_size[j]:
        n_out += 1
        ax_out = j
        boxPoint[j] = box_size[j]

    if n_out > 1:
      continue

    dist = wp.length_sq(boxPoint - axisTip)

    if dist < bestdist:
      bestdist = dist
      bestsegmentpos = wp.float32(i)
      cltype = -2 + i
      clface = ax_out

  # second: consider cases where an edge of the box is closest
  clcorner = wp.int32(-123)  # which corner is the closest
  cledge = wp.int32(-123)  # which axis
  bestboxpos = wp.float32(0.0)

  for i in range(8):
    for j in range(3):
      if i & (1 << j) != 0:
        continue

      c2 = wp.int32(-123)

      # box_pt is the starting point (corner) on the box
      box_pt = wp.cw_mul(
        wp.vec3(
          wp.where(i & 1, 1.0, -1.0),
          wp.where(i & 2, 1.0, -1.0),
          wp.where(i & 4, 1.0, -1.0),
        ),
        box_size,
      )
      box_pt[j] = 0.0

      # find closest point between capsule and the edge
      dif = box_pt - pos

      u = -box_size[j] * dif[j]
      v = wp.dot(halfaxis, dif)
      ma = box_size[j] * box_size[j]
      mb = -box_size[j] * halfaxis[j]
      mc = capsule_half_length * capsule_half_length
      det = ma * mc - mb * mb
      if wp.abs(det) < MJ_MINVAL:
        continue

      idet = 1.0 / det
      # sX : X=1 means middle of segment. X=0 or 2 one or the other end

      x1 = wp.float32((mc * u - mb * v) * idet)
      x2 = wp.float32((ma * v - mb * u) * idet)

      s1 = wp.int32(1)
      s2 = wp.int32(1)

      if x1 > 1:
        x1 = 1.0
        s1 = 2
        x2 = safe_div(v - mb, mc)
      elif x1 < -1:
        x1 = -1.0
        s1 = 0
        x2 = safe_div(v + mb, mc)

      x2_over = x2 > 1.0
      if x2_over or x2 < -1.0:
        if x2_over:
          x2 = 1.0
          s2 = 2
          x1 = safe_div(u - mb, ma)
        else:
          x2 = -1.0
          s2 = 0
          x1 = safe_div(u + mb, ma)

        if x1 > 1:
          x1 = 1.0
          s1 = 2
        elif x1 < -1:
          x1 = -1.0
          s1 = 0

      dif -= halfaxis * x2
      dif[j] += box_size[j] * x1

      # encode relative positions of the closest points
      ct = s1 * 3 + s2

      dif_sq = wp.length_sq(dif)
      if dif_sq < bestdist - MJ_MINVAL:
        bestdist = dif_sq
        bestsegmentpos = x2
        bestboxpos = x1
        # ct<6 means closest point on box is at lower end or middle of edge
        c2 = ct // 6

        clcorner = i + (1 << j) * c2  # index of closest box corner
        cledge = j  # axis index of closest box edge
        cltype = ct  # encoded collision configuration

  best = wp.float32(0.0)

  p = wp.vec2(pos.x, pos.y)
  dd = wp.vec2(halfaxis.x, halfaxis.y)
  s = wp.vec2(box_size[0], box_size[1])
  secondpos = wp.float32(-4.0)

  uu = dd.x * s.y
  vv = dd.y * s.x
  w_neg = dd.x * p.y - dd.y * p.x < 0

  best = wp.float32(-1.0)

  ee1 = uu - vv
  ee2 = uu + vv

  if wp.abs(ee1) > best:
    best = wp.abs(ee1)
    c1 = wp.where((ee1 < 0) == w_neg, 0, 3)

  if wp.abs(ee2) > best:
    best = wp.abs(ee2)
    c1 = wp.where((ee2 > 0) == w_neg, 1, 2)

  if cltype == -4:  # invalid type
    return wp.vec2(wp.inf), mat23f(), mat23f()

  if cltype >= 0 and cltype // 3 != 1:  # closest to a corner of the box
    c1 = axisdir ^ clcorner
    # Calculate relative orientation between capsule and corner
    # There are two possible configurations:
    # 1. Capsule axis points toward/away from corner
    # 2. Capsule axis aligns with a face or edge
    if c1 != 0 and c1 != 7:  # create second contact point
      if c1 == 1 or c1 == 2 or c1 == 4:
        mul = 1
      else:
        mul = -1
        c1 = 7 - c1

      # "de" and "dp" distance from first closest point on the capsule to both ends of it
      # mul is a direction along the capsule's axis

      if c1 == 1:
        ax = 0
        ax1 = 1
        ax2 = 2
      elif c1 == 2:
        ax = 1
        ax1 = 2
        ax2 = 0
      elif c1 == 4:
        ax = 2
        ax1 = 0
        ax2 = 1

      if axis[ax] * axis[ax] > 0.5:  # second point along the edge of the box
        m = 2.0 * safe_div(box_size[ax], wp.abs(halfaxis[ax]))
        secondpos = min(1.0 - wp.float32(mul) * bestsegmentpos, m)
      else:  # second point along a face of the box
        # check for overshoot again
        m = 2.0 * min(
          safe_div(box_size[ax1], wp.abs(halfaxis[ax1])),
          safe_div(box_size[ax2], wp.abs(halfaxis[ax2])),
        )
        secondpos = -min(1.0 + wp.float32(mul) * bestsegmentpos, m)
      secondpos *= wp.float32(mul)

  elif cltype >= 0 and cltype // 3 == 1:  # we are on box's edge
    # Calculate relative orientation between capsule and edge
    # Two possible configurations:
    # - T configuration: c1 = 2^n (no additional contacts)
    # - X configuration: c1 != 2^n (potential additional contacts)
    c1 = axisdir ^ clcorner
    c1 &= 7 - (1 << cledge)  # mask out edge axis to determine configuration

    if c1 == 1 or c1 == 2 or c1 == 4:  # create second contact point
      if cledge == 0:
        ax1 = 1
        ax2 = 2
      if cledge == 1:
        ax1 = 2
        ax2 = 0
      if cledge == 2:
        ax1 = 0
        ax2 = 1
      ax = cledge

      # find which face the capsule has a lower angle, and switch the axis
      if wp.abs(axis[ax1]) > wp.abs(axis[ax2]):
        ax1 = ax2
      ax2 = 3 - ax - ax1

      # mul determines direction along capsule axis for second contact point
      if c1 & (1 << ax2):
        mul = 1
        secondpos = 1.0 - bestsegmentpos
      else:
        mul = -1
        secondpos = 1.0 + bestsegmentpos

      # now find out whether we point towards the opposite side or towards one of the sides
      # and also find the farthest point along the capsule that is above the box

      e1 = 2.0 * safe_div(box_size[ax2], wp.abs(halfaxis[ax2]))
      secondpos = min(e1, secondpos)

      if ((axisdir & (1 << ax)) != 0) == ((c1 & (1 << ax2)) != 0):
        e2 = 1.0 - bestboxpos
      else:
        e2 = 1.0 + bestboxpos

      e1 = box_size[ax] * safe_div(e2, wp.abs(halfaxis[ax]))

      secondpos = min(e1, secondpos)
      secondpos *= wp.float32(mul)

  elif cltype < 0:
    # similarly we handle the case when one capsule's end is closest to a face of the box
    # and find where is the other end pointing to and clamping to the farthest point
    # of the capsule that's above the box
    # if the closest point is inside the box there's no need for a second point

    if clface != -1:  # create second contact point
      mul = wp.where(cltype == -3, 1, -1)
      secondpos = 2.0

      tmp1 = pos - halfaxis * wp.float32(mul)

      for i in range(3):
        if i != clface:
          ha_r = safe_div(wp.float32(mul), halfaxis[i])
          e1 = (box_size[i] - tmp1[i]) * ha_r
          if 0 < e1 and e1 < secondpos:
            secondpos = e1

          e1 = (-box_size[i] - tmp1[i]) * ha_r
          if 0 < e1 and e1 < secondpos:
            secondpos = e1

      secondpos *= wp.float32(mul)

  # create sphere in original orientation at first contact point
  s1_pos_l = pos + halfaxis * bestsegmentpos
  s1_pos_g = box_rot @ s1_pos_l + box_pos

  # collide with sphere using core function
  dist1, pos1, normal1 = sphere_box(s1_pos_g, capsule_radius, box_pos, box_rot, box_size)

  if secondpos > -3:  # secondpos was modified
    s2_pos_l = pos + halfaxis * (secondpos + bestsegmentpos)
    s2_pos_g = box_rot @ s2_pos_l + box_pos

    # collide with sphere using core function
    dist2, pos2, normal2 = sphere_box(s2_pos_g, capsule_radius, box_pos, box_rot, box_size)
  else:
    dist2 = wp.inf
    pos2 = wp.vec3()
    normal2 = wp.vec3()

  return (
    wp.vec2(dist1, dist2),
    mat23f(pos1[0], pos1[1], pos1[2], pos2[0], pos2[1], pos2[2]),
    mat23f(normal1[0], normal1[1], normal1[2], normal2[0], normal2[1], normal2[2]),
  )
