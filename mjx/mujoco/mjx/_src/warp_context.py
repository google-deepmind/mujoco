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
"""Shared helpers for MJX-Warp render contexts."""

from typing import TYPE_CHECKING

import mujoco.mjx.warp as mjxw

if TYPE_CHECKING:
  from mujoco.mjx.warp.render_context import RenderContextPytree


def get_warp_render_context(rc: 'RenderContextPytree'):
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


def get_camera_resolution(warp_rc, cam_id: int) -> tuple[int, int]:
  """Returns the render resolution for a given camera."""
  width = int(warp_rc.cam_res.numpy()[cam_id][0])
  height = int(warp_rc.cam_res.numpy()[cam_id][1])
  return width, height


def require_segmentation_enabled(warp_rc) -> None:
  """Raises if the render context has no segmentation-enabled cameras."""
  if not (warp_rc.seg_adr.numpy() >= 0).any():
    raise ValueError(
        'Render context was not configured with segmentation rendering. '
        'Pass render_seg=True or enable it for at least one camera in '
        'create_render_context.'
    )
