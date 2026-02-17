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

_MJX_RENDER_CONTEXT_COUNTER = 0
_MJX_RENDER_CONTEXT_LOCK = threading.Lock()
_MJX_RENDER_CONTEXT_BUFFERS = {}


def create_render_context(
    mjm: mujoco.MjModel,
    nworld: int,
    **kwargs,
):
  rc = mjw.create_render_context(mjm=mjm, nworld=nworld, **kwargs)
  rc.rgb_data_shape = rc.rgb_data.shape
  rc.depth_data_shape = rc.depth_data.shape
  rc.rgb_data = None
  rc.depth_data = None

  global _MJX_RENDER_CONTEXT_COUNTER
  with _MJX_RENDER_CONTEXT_LOCK:
    _MJX_RENDER_CONTEXT_COUNTER += 1
    key = _MJX_RENDER_CONTEXT_COUNTER
    _MJX_RENDER_CONTEXT_BUFFERS[key] = rc
  return RenderContext(key, _owner=True)
