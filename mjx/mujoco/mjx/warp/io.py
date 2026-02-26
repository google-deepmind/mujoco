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
"""I/O functions for MJX Warp."""

import mujoco
from mujoco.mjx.warp import render_context
import mujoco.mjx.third_party.mujoco_warp as mjw
import warp as wp

_MJX_RENDER_CONTEXT_COUNTER = 0


def _create_context(mjm, nworld, device, **kwargs):
  with wp.ScopedDevice(device):
    ctx = mjw.create_render_context(mjm=mjm, nworld=nworld, **kwargs)
    ctx.rgb_data_shape = ctx.rgb_data.shape
    ctx.depth_data_shape = ctx.depth_data.shape
    ctx.rgb_data = None
    ctx.depth_data = None
  return ctx


def create_render_context(
    mjm: mujoco.MjModel,
    nworld: int,
    devices: list[str | None] | None = None,
    **kwargs,
):
  """Creates a render context using mujoco_warp.create_render_context."""
  global _MJX_RENDER_CONTEXT_COUNTER

  if not devices:
    devices = [None]

  contexts = {}
  default = None
  for d in devices:
    ctx = _create_context(mjm, nworld, d, **kwargs)
    ordinal = wp.get_device(d).ordinal
    contexts[ordinal] = ctx
    if default is None:
      default = ctx

  # pylint: disable=protected-access
  with render_context._MJX_RENDER_CONTEXT_LOCK:
    _MJX_RENDER_CONTEXT_COUNTER += 1
    key = _MJX_RENDER_CONTEXT_COUNTER
    for ordinal, ctx in contexts.items():
      render_context._MJX_RENDER_CONTEXT_BUFFERS[(key, ordinal)] = ctx
    render_context._MJX_RENDER_CONTEXT_BUFFERS[(key, None)] = default
  # pylint: enable=protected-access

  return render_context.RenderContext(
      key=key, contexts=contexts, default=default
  )
