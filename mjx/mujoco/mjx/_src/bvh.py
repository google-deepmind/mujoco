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
"""BVH helpers for MJX."""

from typing import Any
# pylint: disable=g-importing-member
from mujoco.mjx._src.types import Data
from mujoco.mjx._src.types import Impl
from mujoco.mjx._src.types import Model
# pylint: enable=g-importing-member
import mujoco.mjx.warp as mjxw


def refit_bvh(m: Model, d: Data, ctx: Any):
  """Refit the scene BVH for the current pose."""
  if m.impl == Impl.WARP and d.impl == Impl.WARP and mjxw.WARP_INSTALLED:
    from mujoco.mjx.warp import bvh as mjxw_bvh  # pylint: disable=g-import-not-at-top  # pytype: disable=import-error
    return mjxw_bvh.refit_bvh(m, d, ctx)

  raise NotImplementedError('refit_bvh only implemented for MuJoCo Warp.')
