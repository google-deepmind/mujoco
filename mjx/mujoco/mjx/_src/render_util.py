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

import jax
import jax.numpy as jnp


def get_rgb(
    rgb_data: jax.Array,
    cam_id: int,
    width: int,
    height: int,
) -> jax.Array:
  """Unpack uint32 ABGR pixel data into float32 RGB.

  Args:
    rgb_data: Packed render output, shape (nworld, ncam, H*W)
      as uint32.
    cam_id: Camera index to extract.
    width: Image width.
    height: Image height.

  Returns:
    Float32 RGB array with shape (nworld, H, W, 3), values
    in [0, 1].
  """
  packed = rgb_data[:, cam_id]
  r = (packed & 0xFF).astype(jnp.float32) / 255.0
  g = ((packed >> 8) & 0xFF).astype(jnp.float32) / 255.0
  b = ((packed >> 16) & 0xFF).astype(jnp.float32) / 255.0
  rgb = jnp.stack([r, g, b], axis=-1)
  nworld = rgb_data.shape[0]
  return rgb.reshape(nworld, height, width, 3)


def get_depth(
    depth_data: jax.Array,
    cam_id: int,
    width: int,
    height: int,
    depth_scale: float,
) -> jax.Array:
  """Extract and normalize depth data for a camera.

  Args:
    depth_data: Raw depth output, shape (nworld, ncam, H*W)
      as float32.
    cam_id: Camera index to extract.
    width: Image width.
    height: Image height.
    depth_scale: Scale factor for normalizing depth values.

  Returns:
    Float32 depth array with shape (nworld, H, W), clamped
    to [0, 1].
  """
  raw = depth_data[:, cam_id]
  nworld = depth_data.shape[0]
  depth = jnp.clip(raw / depth_scale, 0.0, 1.0)
  return depth.reshape(nworld, height, width)
