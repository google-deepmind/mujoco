# Copyright 2026 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Structural protocol defining the common viewer interface.

StudioApp uses the protocol for convenience methods that accept any viewer.
"""

import dataclasses
from typing import Any
from typing import Protocol
import mujoco
from mujoco.experimental.studio import ux
import numpy as np

GFX_MODES = (
    'classic',
    'classic_headless',
    'opengl',
    'opengl_headless',
    'opengl_software',
    'vulkan',
    'vulkan_software',
    'webgl',
)


@dataclasses.dataclass
class SimToView:
  """A message sent from the simulation to the viewer."""

  model: mujoco.MjModel | None = None
  state: np.ndarray | None = None
  state_sig: int = 0
  user_data: dict[str, Any] = dataclasses.field(default_factory=dict)


@dataclasses.dataclass
class ViewToSim:
  """A message sent from the viewer to the simulation."""

  state: np.ndarray | None = None
  state_sig: int = 0
  reset: bool = False
  send_rate: float = 60.0
  user_data: dict[str, Any] = dataclasses.field(default_factory=dict)


class Viewer(Protocol):
  """Structural interface for any viewer."""

  camera: mujoco.MjvCamera
  perturb: mujoco.MjvPerturb
  vis_options: mujoco.MjvOption
  render_flags: ux.RenderFlags

  def is_running(self) -> bool:
    ...

  def sync(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
    ...

  def stop(self) -> None:
    ...

  def get_drop_file(self) -> str:
    ...
