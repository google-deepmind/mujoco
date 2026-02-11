# Copyright 2026 The Newton Developers
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

import mujoco
import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src.types import ProjectionType

wp.set_module_options({"enable_backward": False})


@wp.kernel
def _convert_texture_data(
  # In:
  width: int,
  adr: int,
  nc: int,
  tex_data_in: wp.array(dtype=wp.uint8),
  # Out:
  tex_data_out: wp.array3d(dtype=float),
):
  """Convert uint8 texture data to vec4 format for efficient sampling."""
  x, y = wp.tid()
  offset = adr + (y * width + x) * nc
  r = tex_data_in[offset + 0] if nc > 0 else wp.uint8(0)
  g = tex_data_in[offset + 1] if nc > 1 else wp.uint8(0)
  b = tex_data_in[offset + 2] if nc > 2 else wp.uint8(0)
  a = wp.uint8(255)

  tex_data_out[y, x, 0] = float(r) * wp.static(1.0 / 255.0)
  tex_data_out[y, x, 1] = float(g) * wp.static(1.0 / 255.0)
  tex_data_out[y, x, 2] = float(b) * wp.static(1.0 / 255.0)
  tex_data_out[y, x, 3] = float(a) * wp.static(1.0 / 255.0)


def create_warp_texture(mjm: mujoco.MjModel, tex_id: int) -> wp.array:
  """Create a Warp texture from a MuJoCo model texture data."""
  tex_adr = mjm.tex_adr[tex_id]
  tex_width = mjm.tex_width[tex_id]
  tex_height = mjm.tex_height[tex_id]
  nchannel = mjm.tex_nchannel[tex_id]
  tex_data = wp.zeros((tex_height, tex_width, 4), dtype=float)

  wp.launch(
    _convert_texture_data,
    dim=(tex_width, tex_height),
    inputs=[tex_width, tex_adr, nchannel, wp.array(mjm.tex_data, dtype=wp.uint8)],
    outputs=[tex_data],
  )
  return wp.Texture2D(tex_data, filter_mode=wp.TextureFilterMode.LINEAR)


@wp.func
def compute_ray(
  # In:
  projection: int,
  fovy: float,
  sensorsize: wp.vec2,
  intrinsic: wp.vec4,
  img_w: int,
  img_h: int,
  px: int,
  py: int,
  znear: float,
) -> wp.vec3:
  """Compute ray direction for a pixel with per-world camera parameters.

  This combines _camera_frustum_bounds and build_primary_rays logic for use
  inside a kernel when camera parameters are batched/randomized across worlds.
  """
  if projection == ProjectionType.ORTHOGRAPHIC:
    return wp.vec3(0.0, 0.0, -1.0)

  aspect = float(img_w) / float(img_h)
  sensor_h = sensorsize[1]

  # Check if we have intrinsics (sensorsize[1] != 0)
  if sensor_h != 0.0:
    fx = intrinsic[0]
    fy = intrinsic[1]
    cx = intrinsic[2]
    cy = intrinsic[3]
    sensor_w = sensorsize[0]

    target_aspect = float(img_w) / float(img_h)
    sensor_aspect = sensor_w / sensor_h
    if target_aspect > sensor_aspect:
      sensor_h = sensor_w / target_aspect
    elif target_aspect < sensor_aspect:
      sensor_w = sensor_h * target_aspect

    inv_fx_znear = znear / fx
    inv_fy_znear = znear / fy
    left = -inv_fx_znear * (sensor_w * 0.5 - cx)
    right = inv_fx_znear * (sensor_w * 0.5 + cx)
    top = inv_fy_znear * (sensor_h * 0.5 - cy)
    bottom = -inv_fy_znear * (sensor_h * 0.5 + cy)
  else:
    fovy_rad = fovy * wp.static(wp.pi / 180.0)
    half_height = znear * wp.tan(0.5 * fovy_rad)
    half_width = half_height * aspect
    left = -half_width
    right = half_width
    top = half_height
    bottom = -half_height

  u = (float(px) + 0.5) / float(img_w)
  v = (float(py) + 0.5) / float(img_h)
  x = left + (right - left) * u
  y = top + (bottom - top) * v

  return wp.normalize(wp.vec3(x, y, -znear))


@wp.func
def pack_rgba_to_uint32(r: float, g: float, b: float, a: float) -> wp.uint32:
  """Pack RGBA values into a single uint32 for efficient memory access."""
  return wp.uint32((int(a) << int(24)) | (int(r) << int(16)) | (int(g) << int(8)) | int(b))
