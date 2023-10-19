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
"""Collision primitives."""

from typing import Tuple

import jax
from jax import numpy as jp
from mujoco.mjx._src import math
# pylint: disable=g-importing-member
from mujoco.mjx._src.collision_base import Contact
from mujoco.mjx._src.collision_base import GeomInfo
# pylint: enable=g-importing-member


def _plane_sphere(
    plane_normal: jax.Array,
    plane_pos: jax.Array,
    sphere_pos: jax.Array,
    radius: jax.Array,
) -> Tuple[jax.Array, jax.Array]:
  """Returns the penetration and contact point between a plane and sphere."""
  cdist = jp.dot(sphere_pos - plane_pos, plane_normal)
  dist = cdist - radius
  pos = sphere_pos - plane_normal * (radius + 0.5 * dist)
  return dist, pos


def plane_sphere(plane: GeomInfo, sphere: GeomInfo) -> Contact:
  """Calculates contact between a plane and a sphere."""
  n = plane.mat[:, 2]
  dist, pos = _plane_sphere(n, plane.pos, sphere.pos, sphere.size[0])
  return jax.tree_map(
      lambda x: jp.expand_dims(x, axis=0), (dist, pos, math.make_frame(n))
  )


def plane_capsule(plane: GeomInfo, cap: GeomInfo) -> Contact:
  """Calculates two contacts between a capsule and a plane."""
  n, axis = plane.mat[:, 2], cap.mat[:, 2]
  # align contact frames with capsule axis
  b, b_norm = math.normalize_with_norm(axis - n * jp.dot(n, axis))
  y, z = jp.array([0.0, 1.0, 0.0]), jp.array([0.0, 0.0, 1.0])
  b = jp.where(b_norm < 0.5, jp.where((-0.5 < n[1]) & (n[1] < 0.5), y, z), b)
  frame = jp.array([[n, b, jp.cross(n, b)]])
  segment = axis * cap.size[1]
  contacts = []
  for offset in [segment, -segment]:
    dist, pos = _plane_sphere(n, plane.pos, cap.pos + offset, cap.size[0])
    dist = jp.expand_dims(dist, axis=0)
    pos = jp.expand_dims(pos, axis=0)
    contacts.append((dist, pos, frame))
  return jax.tree_map(lambda *x: jp.concatenate(x), *contacts)


def _sphere_sphere(
    pos1: jax.Array, radius1: jax.Array, pos2: jax.Array, radius2: jax.Array
) -> Contact:
  """Returns the penetration, contact point, and normal between two spheres."""
  n, dist = math.normalize_with_norm(pos2 - pos1)
  n = jp.where(dist == 0.0, jp.array([1.0, 0.0, 0.0]), n)
  dist = dist - (radius1 + radius2)
  pos = pos1 + n * (radius1 + dist * 0.5)
  return dist, pos, n


def sphere_sphere(s1: GeomInfo, s2: GeomInfo) -> Contact:
  """Calculates contact between two spheres."""
  dist, pos, n = _sphere_sphere(s1.pos, s1.size[0], s2.pos, s2.size[0])
  return jax.tree_map(
      lambda x: jp.expand_dims(x, axis=0), (dist, pos, math.make_frame(n))
  )


def sphere_capsule(sphere: GeomInfo, cap: GeomInfo) -> Contact:
  """Calculates one contact between a sphere and a capsule."""
  axis, length = cap.mat[:, 2], cap.size[1]
  segment = axis * length
  pt = math.closest_segment_point(
      cap.pos - segment, cap.pos + segment, sphere.pos
  )
  dist, pos, n = _sphere_sphere(sphere.pos, sphere.size[0], pt, cap.size[0])
  return jax.tree_map(
      lambda x: jp.expand_dims(x, axis=0), (dist, pos, math.make_frame(n))
  )


def capsule_capsule(cap1: GeomInfo, cap2: GeomInfo) -> Contact:
  """Calculates one contact between two capsules."""
  axis1, length1, axis2, length2 = (
      cap1.mat[:, 2],
      cap1.size[1],
      cap2.mat[:, 2],
      cap2.size[1],
  )
  seg1, seg2 = axis1 * length1, axis2 * length2
  pt1, pt2 = math.closest_segment_to_segment_points(
      cap1.pos - seg1,
      cap1.pos + seg1,
      cap2.pos - seg2,
      cap2.pos + seg2,
  )
  radius1, radius2 = cap1.size[0], cap2.size[0]
  dist, pos, n = _sphere_sphere(pt1, radius1, pt2, radius2)
  return jax.tree_map(
      lambda x: jp.expand_dims(x, axis=0), (dist, pos, math.make_frame(n))
  )

# store ncon as function attributes
plane_sphere.ncon = 1
plane_capsule.ncon = 2
sphere_sphere.ncon = 1
sphere_capsule.ncon = 1
capsule_capsule.ncon = 1
