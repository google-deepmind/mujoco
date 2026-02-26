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
"""MJX Warp render context types and buffer registry."""

import threading

from mujoco.mjx._src import dataclasses as mjx_dataclasses


_MJX_RENDER_CONTEXT_LOCK = threading.Lock()
_MJX_RENDER_CONTEXT_BUFFERS = {}


class RenderContext:
  """MJX render context wrapping one or more warp render contexts.

  Returned by ``io.create_render_context``.  Holds the raw warp contexts
  directly so callers can read camera resolution, buffer addresses, etc.

  Use :meth:`pytree` to obtain the lightweight JAX-compatible handle
  that should be passed into ``jit``/``vmap``-compiled functions such as
  ``render`` and ``refit_bvh``.
  """

  def __init__(self, key, contexts, default):
    self.key = key
    self._contexts = contexts  # {device_ordinal: warp RenderContext}
    self._default = default  # the first warp RenderContext

  def pytree(self):
    """Returns a lightweight JAX pytree for use in jit/vmap."""
    return RenderContextPytree(self.key)

  def __getattr__(self, name):
    """Delegate attribute access to the default warp context."""
    return getattr(self._default, name)

  def __del__(self):
    lock = _MJX_RENDER_CONTEXT_LOCK
    buffers = _MJX_RENDER_CONTEXT_BUFFERS
    if lock is None or buffers is None:
      return
    with lock:
      keys_to_remove = [
          k for k in buffers.keys() if isinstance(k, tuple) and k[0] == self.key
      ]
      for k in keys_to_remove:
        buffers.pop(k, None)


class RenderContextPytree(mjx_dataclasses.PyTreeNode):
  """Minimal JAX pytree holding just the render context key.

  The key is static (aux_data) so JAX doesn't trace it, allowing the
  Warp FFI to receive a concrete int value.
  """

  key: int
