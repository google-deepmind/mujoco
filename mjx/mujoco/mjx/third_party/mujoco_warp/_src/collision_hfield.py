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

from typing import Tuple

import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MAXVAL


@wp.func
def hfield_filter(
  # Model:
  geom_dataid: wp.array(dtype=int),
  geom_aabb: wp.array3d(dtype=wp.vec3),
  geom_rbound: wp.array2d(dtype=float),
  geom_margin: wp.array2d(dtype=float),
  hfield_size: wp.array(dtype=wp.vec4),
  # Data in:
  geom_xpos_in: wp.array2d(dtype=wp.vec3),
  geom_xmat_in: wp.array2d(dtype=wp.mat33),
  # In:
  worldid: int,
  g1: int,
  g2: int,
) -> Tuple[bool, float, float, float, float, float, float]:
  """Filter for height field collisions.

  See MuJoCo mjc_ConvexHField.
  """
  # height field info
  hfdataid = geom_dataid[g1]
  size1 = hfield_size[hfdataid]

  # geom info
  rbound_id = worldid % geom_rbound.shape[0]
  margin_id = worldid % geom_margin.shape[0]

  pos1 = geom_xpos_in[worldid, g1]
  mat1 = geom_xmat_in[worldid, g1]
  mat1T = wp.transpose(mat1)
  pos2 = geom_xpos_in[worldid, g2]
  pos = mat1T @ (pos2 - pos1)
  r2 = geom_rbound[rbound_id, g2]

  # TODO(team): margin?
  margin = wp.max(geom_margin[margin_id, g1], geom_margin[margin_id, g2])

  # box-sphere test: horizontal plane
  for i in range(2):
    if (size1[i] < pos[i] - r2 - margin) or (-size1[i] > pos[i] + r2 + margin):
      return True, wp.inf, wp.inf, wp.inf, wp.inf, wp.inf, wp.inf

  # box-sphere test: vertical direction
  if size1[2] < pos[2] - r2 - margin:  # up
    return True, wp.inf, wp.inf, wp.inf, wp.inf, wp.inf, wp.inf

  if -size1[3] > pos[2] + r2 + margin:  # down
    return True, wp.inf, wp.inf, wp.inf, wp.inf, wp.inf, wp.inf

  mat2 = geom_xmat_in[worldid, g2]
  mat = mat1T @ mat2

  # aabb for geom in height field frame
  xmax = -MJ_MAXVAL
  ymax = -MJ_MAXVAL
  zmax = -MJ_MAXVAL
  xmin = MJ_MAXVAL
  ymin = MJ_MAXVAL
  zmin = MJ_MAXVAL

  aabb_id = worldid % geom_aabb.shape[0]
  center2 = geom_aabb[aabb_id, g2, 0]
  size2 = geom_aabb[aabb_id, g2, 1]

  pos += mat1T @ center2

  sign = wp.vec2(-1.0, 1.0)

  for i in range(2):
    for j in range(2):
      for k in range(2):
        corner_local = wp.vec3(sign[i] * size2[0], sign[j] * size2[1], sign[k] * size2[2])
        corner_hf = mat @ corner_local

        if corner_hf[0] > xmax:
          xmax = corner_hf[0]
        if corner_hf[1] > ymax:
          ymax = corner_hf[1]
        if corner_hf[2] > zmax:
          zmax = corner_hf[2]
        if corner_hf[0] < xmin:
          xmin = corner_hf[0]
        if corner_hf[1] < ymin:
          ymin = corner_hf[1]
        if corner_hf[2] < zmin:
          zmin = corner_hf[2]

  xmax += pos[0]
  xmin += pos[0]
  ymax += pos[1]
  ymin += pos[1]
  zmax += pos[2]
  zmin += pos[2]

  # box-box test
  if (
    (xmin - margin > size1[0])
    or (xmax + margin < -size1[0])
    or (ymin - margin > size1[1])
    or (ymax + margin < -size1[1])
    or (zmin - margin > size1[2])
    or (zmax + margin < -size1[3])
  ):
    return True, wp.inf, wp.inf, wp.inf, wp.inf, wp.inf, wp.inf
  else:
    return False, xmin, xmax, ymin, ymax, zmin, zmax
