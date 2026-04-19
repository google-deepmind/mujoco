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
"""Render helpers for MJX."""

from typing import Any

import jax
import mujoco.mjx.warp as mjxw

from mujoco.mjx._src.warp_context import get_warp_render_context
from mujoco.mjx._src.warp_context import require_segmentation_enabled
# pylint: disable=g-importing-member
from mujoco.mjx._src.types import Data
from mujoco.mjx._src.types import Impl
from mujoco.mjx._src.types import Model
# pylint: enable=g-importing-member


def render(m: Model, d: Data, ctx: Any) -> tuple[jax.Array, jax.Array]:
  """Render packed RGB and depth buffers."""
  if m.impl == Impl.WARP and d.impl == Impl.WARP and mjxw.WARP_INSTALLED:
    from mujoco.mjx.warp import render as mjxw_render

    get_warp_render_context(ctx)
    return mjxw_render.render(m, d, ctx)

  raise NotImplementedError('render only implemented for MuJoCo Warp.')


def render_with_segmentation(
    m: Model, d: Data, ctx: Any
) -> tuple[jax.Array, jax.Array, jax.Array]:
  """Render and return RGB, depth, and packed segmentation outputs."""
  if m.impl == Impl.WARP and d.impl == Impl.WARP and mjxw.WARP_INSTALLED:
    from mujoco.mjx.warp import render as mjxw_render

    warp_rc = get_warp_render_context(ctx)
    require_segmentation_enabled(warp_rc)

    return mjxw_render.render_with_segmentation(m, d, ctx)

  raise NotImplementedError(
      'render_with_segmentation only implemented for MuJoCo Warp.'
  )
