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

from mujoco.mjx.third_party.mujoco_warp._src import types


@wp.func
def mul_quat(u: wp.quat, v: wp.quat) -> wp.quat:
  return wp.quat(
    u[0] * v[0] - u[1] * v[1] - u[2] * v[2] - u[3] * v[3],
    u[0] * v[1] + u[1] * v[0] + u[2] * v[3] - u[3] * v[2],
    u[0] * v[2] - u[1] * v[3] + u[2] * v[0] + u[3] * v[1],
    u[0] * v[3] + u[1] * v[2] - u[2] * v[1] + u[3] * v[0],
  )


@wp.func
def quat_mul_axis(q: wp.quat, axis: wp.vec3f) -> wp.quat:
  """Multiplies a quaternion and an axis."""
  return wp.quat(
    -q[1] * axis[0] - q[2] * axis[1] - q[3] * axis[2],
    q[0] * axis[0] + q[2] * axis[2] - q[3] * axis[1],
    q[0] * axis[1] + q[3] * axis[0] - q[1] * axis[2],
    q[0] * axis[2] + q[1] * axis[1] - q[2] * axis[0],
  )


@wp.func
def rot_vec_quat(vec: wp.vec3, quat: wp.quat) -> wp.vec3:
  s, u = quat[0], wp.vec3(quat[1], quat[2], quat[3])
  r = 2.0 * (wp.dot(u, vec) * u) + (s * s - wp.dot(u, u)) * vec
  r = r + 2.0 * s * wp.cross(u, vec)
  return r


@wp.func
def axis_angle_to_quat(axis: wp.vec3, angle: float) -> wp.quat:
  s, c = wp.sin(angle * 0.5), wp.cos(angle * 0.5)
  axis = axis * s
  return wp.quat(c, axis[0], axis[1], axis[2])


@wp.func
def quat_to_mat(quat: wp.quat) -> wp.mat33:
  """Converts a quaternion into a 9-dimensional rotation matrix."""
  vec = wp.vec4(quat[0], quat[1], quat[2], quat[3])
  q = wp.outer(vec, vec)

  return wp.mat33(
    q[0, 0] + q[1, 1] - q[2, 2] - q[3, 3],
    2.0 * (q[1, 2] - q[0, 3]),
    2.0 * (q[1, 3] + q[0, 2]),
    2.0 * (q[1, 2] + q[0, 3]),
    q[0, 0] - q[1, 1] + q[2, 2] - q[3, 3],
    2.0 * (q[2, 3] - q[0, 1]),
    2.0 * (q[1, 3] - q[0, 2]),
    2.0 * (q[2, 3] + q[0, 1]),
    q[0, 0] - q[1, 1] - q[2, 2] + q[3, 3],
  )


@wp.func
def quat_inv(quat: wp.quat) -> wp.quat:
  return wp.quat(quat[0], -quat[1], -quat[2], -quat[3])


@wp.func
def inert_vec(i: types.vec10, v: wp.spatial_vector) -> wp.spatial_vector:
  """mju_mulInertVec: multiply 6D vector (rotation, translation) by 6D inertia matrix."""
  return wp.spatial_vector(
    i[0] * v[0] + i[3] * v[1] + i[4] * v[2] - i[8] * v[4] + i[7] * v[5],
    i[3] * v[0] + i[1] * v[1] + i[5] * v[2] + i[8] * v[3] - i[6] * v[5],
    i[4] * v[0] + i[5] * v[1] + i[2] * v[2] - i[7] * v[3] + i[6] * v[4],
    i[8] * v[1] - i[7] * v[2] + i[9] * v[3],
    i[6] * v[2] - i[8] * v[0] + i[9] * v[4],
    i[7] * v[0] - i[6] * v[1] + i[9] * v[5],
  )


@wp.func
def motion_cross(u: wp.spatial_vector, v: wp.spatial_vector) -> wp.spatial_vector:
  """Cross product of two motions."""
  u0 = wp.vec3(u[0], u[1], u[2])
  u1 = wp.vec3(u[3], u[4], u[5])
  v0 = wp.vec3(v[0], v[1], v[2])
  v1 = wp.vec3(v[3], v[4], v[5])

  ang = wp.cross(u0, v0)
  vel = wp.cross(u1, v0) + wp.cross(u0, v1)

  return wp.spatial_vector(ang, vel)


@wp.func
def motion_cross_force(v: wp.spatial_vector, f: wp.spatial_vector) -> wp.spatial_vector:
  """Cross product of a motion and a force."""
  v0 = wp.vec3(v[0], v[1], v[2])
  v1 = wp.vec3(v[3], v[4], v[5])
  f0 = wp.vec3(f[0], f[1], f[2])
  f1 = wp.vec3(f[3], f[4], f[5])

  ang = wp.cross(v0, f0) + wp.cross(v1, f1)
  vel = wp.cross(v0, f1)

  return wp.spatial_vector(ang, vel)


@wp.func
def quat_to_vel(quat: wp.quat) -> wp.vec3:
  axis = wp.vec3(quat[1], quat[2], quat[3])
  sin_a_2 = wp.norm_l2(axis)

  if sin_a_2 == 0.0:
    return wp.vec3(0.0)

  speed = 2.0 * wp.atan2(sin_a_2, quat[0])
  # when axis-angle is larger than pi, rotation is in the opposite direction
  if speed > wp.pi:
    speed -= 2.0 * wp.pi

  return axis * speed / sin_a_2


@wp.func
def quat_sub(qa: wp.quat, qb: wp.quat) -> wp.vec3:
  """Subtract quaternions, express as 3D velocity: qb*quat(res) = qa."""
  # qdif = neg(qb)*qa
  qneg = wp.quat(qb[0], -qb[1], -qb[2], -qb[3])
  qdif = mul_quat(qneg, qa)

  # convert to 3D velocity
  return quat_to_vel(qdif)


@wp.func
def quat_integrate(q: wp.quat, v: wp.vec3, dt: float) -> wp.quat:
  """Integrates a quaternion given angular velocity and dt."""
  norm_ = wp.length(v)
  v = wp.normalize(v)  # does that need proper zero gradient handling?
  angle = dt * norm_

  q_res = axis_angle_to_quat(v, angle)
  q = wp.normalize(q)
  q_res = mul_quat(q, q_res)

  return wp.normalize(q_res)


@wp.func
def orthogonals(a: wp.vec3):
  y = wp.vec3(0.0, 1.0, 0.0)
  z = wp.vec3(0.0, 0.0, 1.0)
  b = wp.where((-0.5 < a[1]) and (a[1] < 0.5), y, z)
  b = b - a * wp.dot(a, b)
  b = wp.normalize(b)
  if wp.length(a) == 0.0:
    b = wp.vec3(0.0, 0.0, 0.0)
  c = wp.cross(a, b)

  return b, c


@wp.func
def orthonormal(normal: wp.vec3) -> wp.vec3:
  if wp.abs(normal[0]) < wp.abs(normal[1]) and wp.abs(normal[0]) < wp.abs(normal[2]):
    dir = wp.vec3(1.0 - normal[0] * normal[0], -normal[0] * normal[1], -normal[0] * normal[2])
  elif wp.abs(normal[1]) < wp.abs(normal[2]):
    dir = wp.vec3(-normal[1] * normal[0], 1.0 - normal[1] * normal[1], -normal[1] * normal[2])
  else:
    dir = wp.vec3(-normal[2] * normal[0], -normal[2] * normal[1], 1.0 - normal[2] * normal[2])
  dir, _ = gjk_normalize(dir)
  return dir


@wp.func
def orthonormal_to_z(normal: wp.vec3) -> wp.vec3:
  if wp.abs(normal[0]) < wp.abs(normal[1]):
    dir = wp.vec3(1.0 - normal[0] * normal[0], -normal[0] * normal[1], -normal[0] * normal[2])
  else:
    dir = wp.vec3(-normal[1] * normal[0], 1.0 - normal[1] * normal[1], -normal[1] * normal[2])
  dir, _ = gjk_normalize(dir)
  return dir


@wp.func
def gjk_normalize(a: wp.vec3):
  norm = wp.length(a)
  if norm > 1e-8 and norm < 1e12:
    return a / norm, True
  return a, False


@wp.func
def make_frame(a: wp.vec3):
  a = wp.normalize(a)
  b, c = orthogonals(a)

  # fmt: off
  return wp.mat33(
    a.x, a.y, a.z,
    b.x, b.y, b.z,
    c.x, c.y, c.z
  )
  # fmt: on


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


@wp.func
def safe_div(x: Any, y: Any) -> Any:
  return x / wp.where(y != 0.0, y, types.MJ_MINVAL)


@wp.func
def upper_tri_index(n: int, i: int, j: int) -> int:
  """Returns index of a_ij = a_ji in upper triangular matrix (excluding diagonal)."""
  return (i * (2 * n - i - 3)) // 2 + j - 1


@wp.func
def upper_trid_index(n: int, i: int, j: int) -> int:
  """Returns index of a_ij = a_ji in upper triangular matrix (including diagonal)."""
  if j < i:
    i, j = j, i
  return (i * (2 * n - i - 1)) // 2 + j
