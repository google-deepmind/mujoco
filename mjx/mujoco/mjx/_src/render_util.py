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

from typing import TYPE_CHECKING

import jax
import jax.numpy as jnp

import mujoco.mjx.warp as mjxw

if TYPE_CHECKING:
  from mujoco.mjx.warp.render_context import RenderContextPytree


def _get_warp_render_context(rc: 'RenderContextPytree'):
  """Validates and returns the backing Warp render context."""
  if not mjxw.WARP_INSTALLED:
    raise RuntimeError('Warp not installed.')

  from mujoco.mjx.warp import render_context as mjxw_rc

  if not isinstance(rc, mjxw_rc.RenderContextPytree):
    raise TypeError(
        f'Expected RenderContextPytree, got {type(rc).__name__}.'
        ' Use rc.pytree() to get the JAX-compatible handle.'
    )

  # pylint: disable=protected-access
  return mjxw_rc._MJX_RENDER_CONTEXT_BUFFERS[(rc.key, None)]


def _get_camera_resolution(warp_rc, cam_id: int) -> tuple[int, int]:
  """Returns the render resolution for a given camera."""
  width = int(warp_rc.cam_res.numpy()[cam_id][0])
  height = int(warp_rc.cam_res.numpy()[cam_id][1])
  return width, height


def get_rgb(
    rc: 'RenderContextPytree',
    cam_id: int,
    rgb_data: jax.Array,
) -> jax.Array:
  """Unpack uint32 ABGR pixel data into float32 RGB.

  Args:
    rc: RenderContextPytree.
    cam_id: Camera index to extract.
    rgb_data: Packed render output, shape (..., total_pixels) as uint32.

  Returns:
    Float32 RGB array with shape (..., H, W, 3), values in [0, 1].
    Any leading batch axes in `rgb_data` are preserved.

  Raises:
    RuntimeError: If Warp is not installed.
  """
  warp_rc = _get_warp_render_context(rc)
  rgb_adr = int(warp_rc.rgb_adr.numpy()[cam_id])
  width, height = _get_camera_resolution(warp_rc, cam_id)

  packed = jax.lax.dynamic_slice_in_dim(
      rgb_data, rgb_adr, width * height, axis=rgb_data.ndim - 1
  )

  b = (packed & 0xFF).astype(jnp.float32) / 255.0
  g = ((packed >> 8) & 0xFF).astype(jnp.float32) / 255.0
  r = ((packed >> 16) & 0xFF).astype(jnp.float32) / 255.0
  rgb = jnp.stack([r, g, b], axis=-1)
  return rgb.reshape(packed.shape[:-1] + (height, width, 3))


def get_depth(
    rc: 'RenderContextPytree',
    cam_id: int,
    depth_data: jax.Array,
    depth_scale: float,
) -> jax.Array:
  """Extract and normalize depth data for a camera.

  Args:
    rc: RenderContextPytree.
    cam_id: Camera index to extract.
    depth_data: Raw depth output, shape (..., total_pixels) as float32.
    depth_scale: Scale factor for normalizing depth values.

  Returns:
    Float32 depth array with shape (..., H, W, 1), clamped to [0, 1].
    Any leading batch axes in `depth_data` are preserved.

  Raises:
    RuntimeError: If Warp is not installed.
  """
  warp_rc = _get_warp_render_context(rc)
  depth_adr = int(warp_rc.depth_adr.numpy()[cam_id])
  width, height = _get_camera_resolution(warp_rc, cam_id)

  raw = jax.lax.dynamic_slice_in_dim(
      depth_data, depth_adr, width * height, axis=depth_data.ndim - 1
  )

  depth = jnp.clip(raw / depth_scale, 0.0, 1.0)
  return depth.reshape(raw.shape[:-1] + (height, width, 1))


def get_segmentation(
    rc: 'RenderContextPytree',
    cam_id: int,
    seg_data: jax.Array,
) -> jax.Array:
  """Extract raw geom IDs for a camera.

  Args:
    rc: RenderContextPytree.
    cam_id: Camera index to extract.
    seg_data: Packed segmentation output, shape (..., total_pixels) as integers.

  Returns:
    Integer segmentation array with shape (..., H, W).
    Any leading batch axes in `seg_data` are preserved.

  Raises:
    RuntimeError: If Warp is not installed.
    ValueError: If segmentation is not enabled for the selected camera.
  """
  warp_rc = _get_warp_render_context(rc)
  seg_adr = int(warp_rc.seg_adr.numpy()[cam_id])
  if seg_adr < 0:
    raise ValueError(
        f'Camera {cam_id} was not configured with segmentation rendering.'
    )

  width, height = _get_camera_resolution(warp_rc, cam_id)
  packed = jax.lax.dynamic_slice_in_dim(
      seg_data, seg_adr, width * height, axis=seg_data.ndim - 1
  )
  return packed.reshape(packed.shape[:-1] + (height, width))
