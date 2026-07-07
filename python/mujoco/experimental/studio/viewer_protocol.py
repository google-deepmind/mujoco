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
"""Base class and configuration for any viewer."""

import abc
import copy
import dataclasses
import enum
from typing import Any
import mujoco
from mujoco.experimental.studio import endpoints
from mujoco.experimental.studio import handler_registry
from mujoco.experimental.studio import messages
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


class ViewerMode(enum.StrEnum):
  """Determines where the viewer is rendered."""

  NATIVE = 'native'
  WEB = 'web'


# -----------------------------------------------------------------------------
# Viewer configuration.
# -----------------------------------------------------------------------------


@dataclasses.dataclass
class ViewerConfig:
  """Common configuration for creating a viewer window."""

  title: str = ''
  width: int = 1200
  height: int = 800
  gfx: str = ''
  viewer_mode: ViewerMode = ViewerMode.NATIVE


# Legacy message types kept for backward compatibility.
# Will be removed when callers are migrated.


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


# -----------------------------------------------------------------------------
# Base class for any viewer.
# -----------------------------------------------------------------------------


@dataclasses.dataclass(frozen=True)
class ViewerInitEvent(messages.Event):
  """Lifecycle event dispatched once when the concrete Viewer is initialized.

  Handlers that need access to the Viewer should handle this event
  and cache the reference.
  """

  viewer: 'Viewer'


class Viewer(abc.ABC):
  """Base class for any viewer.

  Owns the communication endpoint, handler registry and core visualization
  objects. The application is rendered by calling ``sync()``.
  """

  def __init__(
      self,
      config: ViewerConfig,
      endpoint: endpoints.ViewerEndpoint,
      *,
      model: mujoco.MjModel | None = None,
      model_path: str = '',
      handlers: list[Any] | None = None,
      camera: mujoco.MjvCamera | None = None,
      vis_options: mujoco.MjvOption | None = None,
      perturb: mujoco.MjvPerturb | None = None,
      render_flags: ux.RenderFlags | None = None,
      extra_geoms: list[mujoco.MjvGeom] | None = None,
  ) -> None:
    """Initializes the Viewer.

    Args:
      config: Viewer window configuration.
      endpoint: The viewer endpoint for communication with the sim side.
      model: Optional initial MjModel. If None, an empty model is created from
        an empty MjSpec. The Viewer deep-copies this model and creates its own
        MjData.
      model_path: Optional path to the model file.
      handlers: Optional list of handler instances for viewer-side processing.
      camera: Camera parameters. Internal object is created if None.
      vis_options: Visualization options. Internal object is created if None.
      perturb: Perturbation parameters. Internal object is created if None.
      render_flags: Render flags. Internal object is created if None.
      extra_geoms: List of extra geoms. Internal list is created if None.
    """
    self.config = config
    self._endpoint = endpoint
    self._is_running = True

    # Viewer-owned model and data.
    if model is None:
      model = mujoco.MjSpec().compile()
    self.model: mujoco.MjModel
    self.data: mujoco.MjData
    self.model_path: str = ''
    self.load_model(model, model_path)

    # Visual state.
    self.camera = camera or mujoco.MjvCamera()
    self.cam_speed = 0.001
    self.perturb = perturb or mujoco.MjvPerturb()
    self.vis_options = vis_options or mujoco.MjvOption()
    self.extra_geoms = extra_geoms or []
    if render_flags is not None:
      self.render_flags = render_flags
    else:
      self.render_flags = ux.RenderFlags()
      # Initted to match mujoco/src/engine/engine_vis_init.c
      self.render_flags.flags = [1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1]

    # Handler infrastructure.
    all_handlers = [self] + list(handlers or [])
    self.handlers = handler_registry.HandlerRegistry(all_handlers)

  def close(self) -> None:
    """Closes the viewer, sends an exit event and shuts down the endpoint."""
    if self._is_running:
      self._is_running = False
      try:
        self.send_to_sim(messages.ExitEvent())
      except Exception:  # pylint: disable=broad-exception-caught
        pass  # Ignore exceptions, the sim may have already closed.
      self._endpoint.close()

  def is_running(self) -> bool:
    """Returns True while the viewer has not been closed."""
    return self._is_running

  def send_to_sim(self, message: messages.Message) -> None:
    """Sends a message to the simulation process."""
    self._endpoint.send_to_sim(message)

  def get_sim_events(self) -> list[messages.Event]:
    """Returns all pending events from the simulation."""
    return self._endpoint.get_sim_events()

  def get_sim_snapshots(self) -> list[messages.Snapshot]:
    """Returns all pending latest snapshots from the simulation, one per type."""
    return self._endpoint.get_sim_snapshots()

  def dispatch(self, message: messages.Message) -> None:
    """Dispatches a message to registered handlers in priority order."""
    self.handlers.dispatch(message)

  def load_model(self, model: mujoco.MjModel, model_path: str = '') -> None:
    """Deep-copies a model and creates fresh data for the viewer."""
    self.model_path = model_path
    self.model = copy.deepcopy(model)
    self.data = mujoco.MjData(self.model)
    assert id(self.model) != id(model)
    mujoco.mj_forward(self.model, self.data)

  @messages.handler(priority=messages.Priority.CRITICAL)
  def _on_model(self, event: messages.ModelEvent) -> bool:
    """Deep-copies the incoming model so the Viewer owns its data."""
    self.load_model(event.model, event.path)
    self.extra_geoms.clear()
    return False  # Do not consume; let other handlers see the event.

  @messages.handler(priority=messages.Priority.CRITICAL)
  def _on_state(self, event: messages.StateSnapshot) -> bool:
    """Applies incoming simulation state to the viewer's model/data."""
    state_size = mujoco.mj_stateSize(self.model, event.state_sig)
    if len(event.state) == state_size:
      mujoco.mj_setState(self.model, self.data, event.state, event.state_sig)
      mujoco.mj_forward(self.model, self.data)
    return False  # Do not consume; let other handlers see the event.

  @abc.abstractmethod
  def sync(self) -> None:
    """Renders the scene using the viewer's current model and data."""
    ...

  @abc.abstractmethod
  def get_drop_file(self) -> str:
    ...

  @abc.abstractmethod
  def upload_image(
      self, tex_id: int, img: str | bytes, width: int, height: int, bpp: int
  ) -> int:
    ...


# -----------------------------------------------------------------------------
# Standalone viewer loop.
# -----------------------------------------------------------------------------


def run_viewer_loop(viewer: Viewer) -> None:
  """Minimal viewer loop: process sim messages, dispatch lifecycle events, sync.

  Runs until the viewer window is closed or an exit event is received.
  On exit, closes the viewer (which sends an ExitEvent to the sim side).

  Args:
    viewer: A Viewer that owns the endpoint and handler registry.
  """
  while viewer.is_running():
    # Process incoming messages.
    for event in viewer.get_sim_events():
      viewer.dispatch(event)
    for snapshot in viewer.get_sim_snapshots():
      viewer.dispatch(snapshot)

    # Dispatch lifecycle events.
    viewer.dispatch(messages.UpdateEvent())
    viewer.dispatch(messages.BuildGuiEvent())

    # Render the scene.
    viewer.sync()

  viewer.close()
