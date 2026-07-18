# Copyright 2024 NVIDIA CORPORATION.
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

import warp as wp
from typing import Optional, Tuple, Union

import jax
import jax.numpy as jp
import numpy as np
from mujoco.mjx._src.jax_warp import jax_kernel
from mujoco.mjx._src.collision_base import GeomInfo

wp.init()

@wp.func
def orthogonals(a: wp.vec3):
  y, z = wp.vec3(0., 1., 0.), wp.vec3(0., 0., 1.)
  b = wp.select((-0.5 < a[1]) and (a[1] < 0.5), z, y)
  b = b - a * wp.dot(a, b)
  b = wp.normalize(b)
  return b, wp.cross(a, b)

@wp.func
def make_frame(a: wp.vec3) -> wp.mat33:
  """Makes a right-handed 3D frame given a direction."""
  a = wp.normalize(a)
  b, c = orthogonals(a)
  return wp.mat33(a, b, c)

@wp.func
def closest_segment_point(a: wp.vec3, b: wp.vec3, pt: wp.vec3):
  """Returns the closest point on the a-b line segment to a point pt."""
  ab = b - a
  t = wp.dot(pt - a, ab) / (wp.dot(ab, ab) + 1e-6)
  return a + wp.clamp(t, 0.0, 1.0) * ab

@wp.func
def closest_segment_point_and_dist(a: wp.vec3, b: wp.vec3, pt: wp.vec3):
  """Returns closest point on the line segment and the distance squared."""
  closest = closest_segment_point(a, b, pt)
  dist = wp.dot(pt - closest, pt - closest)
  return closest, dist

@wp.func
def closest_segment_to_segment_points(a0: wp.vec3, a1: wp.vec3, b0: wp.vec3, b1: wp.vec3):
  """Returns closest points between two line segments."""
  # Gets the closest segment points by first finding the closest points
  # between two lines. Points are then clipped to be on the line segments
  # and edge cases with clipping are handled.
  len_a = wp.length(a1 - a0)
  dir_a = wp.normalize(a1 - a0)
  len_b = wp.length(b1 - b0)
  dir_b = wp.normalize(b1 - b0)

  # Segment mid-points.
  half_len_a = len_a * 0.5
  half_len_b = len_b * 0.5
  a_mid = a0 + dir_a * half_len_a
  b_mid = b0 + dir_b * half_len_b

  # Translation between two segment mid-points.
  trans = a_mid - b_mid

  # Parametrize points on each line as follows:
  #  point_on_a = a_mid + t_a * dir_a
  #  point_on_b = b_mid + t_b * dir_b
  # and analytically minimize the distance between the two points.
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

  # Resolve edge cases where both closest points are clipped to the segment
  # endpoints by recalculating the closest segment points for the current
  # clipped points, and then picking the pair of points with smallest
  # distance. An example of this edge case is when lines intersect but line
  # segments don't.
  new_a, d1 = closest_segment_point_and_dist(a0, a1, best_b)
  new_b, d2 = closest_segment_point_and_dist(b0, b1, best_a)
  best_a = wp.select(d1 < d2, best_a, new_a)
  best_b = wp.select(d1 < d2, new_b, best_b)

  return best_a, best_b

@wp.func
def _sphere_sphere(pos1: wp.vec3, radius1: float, pos2: wp.vec3, radius2: float):
  """Returns the penetration, contact point, and normal between two spheres."""
  dist = wp.length(pos2 - pos1)
  n = wp.normalize(pos2 - pos1)
  n = wp.select(dist == 0.0, n, wp.vec3(1.0, 0.0, 0.0))
  dist = dist - (radius1 + radius2)
  pos = pos1 + n * (radius1 + dist * 0.5)
  return dist, pos, n

@wp.func
def capsule_capsule(cap1_pos: wp.vec3, cap1_mat: wp.mat33, cap1_size: wp.vec3,
                    cap2_pos: wp.vec3, cap2_mat: wp.mat33, cap2_size: wp.vec3):
  """Calculates one contact between two capsules."""
  axis1, length1, axis2, length2 = (
      wp.transpose(cap1_mat)[2],
      cap1_size[1],
      wp.transpose(cap2_mat)[2],
      cap2_size[1],
  )
  seg1, seg2 = axis1 * length1, axis2 * length2
  pt1, pt2 = closest_segment_to_segment_points(
      cap1_pos - seg1,
      cap1_pos + seg1,
      cap2_pos - seg2,
      cap2_pos + seg2,
  )
  radius1, radius2 = cap1_size[0], cap2_size[0]
  dist, pos, n = _sphere_sphere(pt1, radius1, pt2, radius2)
  return dist, pos, make_frame(n)

@wp.kernel
def capsule_capsule_kernel(
      cap1_pos: wp.array(dtype=wp.vec3),
      cap1_mat: wp.array(dtype=wp.mat33),
      cap1_size: wp.array(dtype=wp.vec3),
      cap2_pos: wp.array(dtype=wp.vec3),
      cap2_mat: wp.array(dtype=wp.mat33),
      cap2_size: wp.array(dtype=wp.vec3),
      dist_out: wp.array(dtype=float),
      pos_out: wp.array(dtype=wp.vec3),
      frame_out: wp.array(dtype=wp.mat33)):
  tid = wp.tid()
  dist, pos, frame = capsule_capsule(cap1_pos[tid], cap1_mat[tid], cap1_size[tid],
                                     cap2_pos[tid], cap2_mat[tid], cap2_size[tid])
  dist_out[tid] = dist
  pos_out[tid] = pos
  frame_out[tid] = wp.transpose(frame)

capsule_capsule_warp = jax_kernel(capsule_capsule_kernel)
# @jax.jit

def capsule_capsule_opt(g1: GeomInfo, g2: GeomInfo):
  # Hack: Lift to operate on 1-element array and leave it to the batching machinery
  # to lift to multiple dimensions.
  (jpos1, jmat1, jsize1) = jax.tree_map(lambda x: jp.expand_dims(x, axis=0), jax.tree.flatten(g1)[0])
  (jpos2, jmat2, jsize2) = jax.tree_map(lambda x: jp.expand_dims(x, axis=0), jax.tree.flatten(g2)[0])
  return capsule_capsule_warp(jpos1, jmat1, jsize1, jpos2, jmat2, jsize2)

capsule_capsule_opt.ncon = 1
