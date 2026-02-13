# Copyright 2026 DeepMind Technologies Limited
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
"""JAX render utilities for unpacking render output from MuJoCo Warp."""

import typing
from typing import Any

import jax
import jax.numpy as jnp


def get_rgb(
    rc: Any,
    rgb_data: jax.Array,
    cam_id: int,
) -> jax.Array:
  """Unpack uint32 ABGR pixel data into float32 RGB.

  Args:
    rc: The RenderContext handle.
    rgb_data: Packed render output, shape (nworld, total_pixels)
      as uint32.
    cam_id: Camera index to extract.

  Returns:
    Float32 RGB array with shape (nworld, H, W, 3), values
    in [0, 1].
  """
  import mujoco.mjx.warp.render as mjxw_render  # pylint: disable=g-import-not-at-top
  warp_rc = mjxw_render._MJX_RENDER_CONTEXT_BUFFERS[rc.key]
  rgb_adr = int(warp_rc.rgb_adr.numpy()[cam_id])
  width = int(warp_rc.cam_res.numpy()[cam_id][0])
  height = int(warp_rc.cam_res.numpy()[cam_id][1])

  packed = jax.lax.dynamic_slice_in_dim(
      rgb_data, rgb_adr, width * height, axis=1
  )

  r = (packed & 0xFF).astype(jnp.float32) / 255.0
  g = ((packed >> 8) & 0xFF).astype(jnp.float32) / 255.0
  b = ((packed >> 16) & 0xFF).astype(jnp.float32) / 255.0
  rgb = jnp.stack([r, g, b], axis=-1)
  nworld = rgb_data.shape[0]
  return rgb.reshape(nworld, height, width, 3)


def get_depth(
    rc: Any,
    depth_data: jax.Array,
    cam_id: int,
    depth_scale: float,
) -> jax.Array:
  """Extract and normalize depth data for a camera.

  Args:
    rc: The RenderContext handle.
    depth_data: Raw depth output, shape (nworld, total_pixels)
      as float32.
    cam_id: Camera index to extract.
    depth_scale: Scale factor for normalizing depth values.

  Returns:
    Float32 depth array with shape (nworld, H, W), clamped
    to [0, 1].
  """
  import mujoco.mjx.warp.render as mjxw_render  # pylint: disable=g-import-not-at-top
  warp_rc = mjxw_render._MJX_RENDER_CONTEXT_BUFFERS[rc.key]
  depth_adr = int(warp_rc.depth_adr.numpy()[cam_id])
  width = int(warp_rc.cam_res.numpy()[cam_id][0])
  height = int(warp_rc.cam_res.numpy()[cam_id][1])

  raw = jax.lax.dynamic_slice_in_dim(
      depth_data, depth_adr, width * height, axis=1
  )

  nworld = depth_data.shape[0]
  depth = jnp.clip(raw / depth_scale, 0.0, 1.0)
  return depth.reshape(nworld, height, width)

