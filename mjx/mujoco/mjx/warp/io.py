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

import threading

import mujoco
from mujoco.mjx.warp.types import RenderContext
import mujoco.mjx.third_party.mujoco_warp as mjw
import warp as wp

_MJX_RENDER_CONTEXT_COUNTER = 0
_MJX_RENDER_CONTEXT_LOCK = threading.Lock()
_MJX_RENDER_CONTEXT_BUFFERS = {}


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
  global _MJX_RENDER_CONTEXT_COUNTER

  if not devices:
    devices = [None]

  contexts = [_create_context(mjm, nworld, d, **kwargs) for d in devices]

  with _MJX_RENDER_CONTEXT_LOCK:
    _MJX_RENDER_CONTEXT_COUNTER += 1
    key = _MJX_RENDER_CONTEXT_COUNTER
    for d, ctx in zip(devices, contexts):
      ordinal = wp.get_device(d).ordinal
      _MJX_RENDER_CONTEXT_BUFFERS[(key, ordinal)] = ctx
    if (key, None) not in _MJX_RENDER_CONTEXT_BUFFERS:
      # save the first context as the default context
      _MJX_RENDER_CONTEXT_BUFFERS[(key, None)] = contexts[0]
  return RenderContext(key, _owner=True)
