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
"""Simulation application handle and event hooks."""

from typing import Callable, Protocol
import mujoco
from mujoco.experimental.studio import endpoints
from mujoco.experimental.studio import messages
from mujoco.experimental.studio import sim as _sim
import numpy as np

# -----------------------------------------------------------------------------
# Custom event handling API.
# Note: See viewer_app.py for hooks to customize the viewer behavior.
# -----------------------------------------------------------------------------


class SimEventHandler(Protocol):
  """Invoked once per event received from the viewer."""

  def handle(self, event: messages.Event) -> bool:
    """Return True if event was consumed, False to allow further processing."""
    ...


class ViewerHandle:
  """A handle held by the simulation to sync the simulation with the viewer."""

  def __init__(
      self,
      sim_endpoint: endpoints.SimEndpoint,
      *,
      sim_event_handler: SimEventHandler | None = None,
      is_alive_fn: Callable[[], bool] | None = None,
  ):
    """Initializes the handle.

    Args:
      sim_endpoint: The endpoint to use for communication with the viewer.
      sim_event_handler: Optional handler for viewer-to-sim events.
      is_alive_fn: Optional function called to check if the viewer is still
        alive/responsive. If not provided, the viewer is assumed to be running
        until `close()` is called.
    """

    self._sim_endpoint = sim_endpoint
    self._is_running = True
    self._is_alive_fn = is_alive_fn
    self._sim_event_handler = sim_event_handler

  def close(self) -> None:
    """Signals the viewer to exit and closes the sim endpoint, releasing resources."""
    if self._is_running:
      self._is_running = False
      try:
        self.send_to_viewer(messages.ExitEvent())
      except Exception:  # pylint: disable=broad-exception-caught
        pass  # Ignore exceptions, the viewer may have already closed.
    self._sim_endpoint.close()

  def __enter__(self):
    return self

  def __exit__(self, exc_type, exc_val, exc_tb):
    self.close()

  def is_running(self) -> bool:
    """Returns True while the viewer is open."""
    if self._is_alive_fn is not None and not self._is_alive_fn():
      self.close()
    return self._is_running

  def send_to_viewer(self, message: messages.Message) -> None:
    self._sim_endpoint.send_to_viewer(message)

  def sync(
      self,
      model: mujoco.MjModel | None,
      data: mujoco.MjData | None,
      step_control: _sim.StepControl,
  ) -> tuple[mujoco.MjModel | None, mujoco.MjData | None, _sim.StepControl]:
    """Syncs the simulation with the viewer and returns the updated sim state.

    This method processes incoming events from the viewer, updates the sim state
    accordingly, and sends the current simulation state to the viewer as a
    snapshot.

    Args:
      model: The current model.
      data: The current data.
      step_control: The current step control state.

    Returns:
      The updated model, data, and step control state.
    """

    integration_sig = int(mujoco.mjtState.mjSTATE_INTEGRATION)

    # Process incoming events from the viewer.
    for event in self._sim_endpoint.get_viewer_events():
      if (
          self._sim_event_handler is not None
          and self._sim_event_handler.handle(event)
      ):
        continue
      if isinstance(event, messages.ModelEvent):
        # A new model was loaded in the viewer (e.g. via file drop).
        model = event.model
        data = mujoco.MjData(event.model)
        mujoco.mj_forward(model, data)
        step_control = _sim.StepControl()
      elif isinstance(event, messages.PerturbEvent):
        if model is not None and data is not None:
          state_size = mujoco.mj_stateSize(model, event.state_sig)
          if len(event.state) == state_size:
            mujoco.mj_setState(model, data, event.state, event.state_sig)
      elif isinstance(event, messages.ResetEvent):
        if model is not None:
          assert data is not None
          mujoco.mj_resetData(model, data)
          mujoco.mj_forward(model, data)
      elif isinstance(event, messages.ExitEvent):
        self._is_running = False

    # Process incoming snapshots from the viewer.
    for snapshot in self._sim_endpoint.get_viewer_snapshots():
      if isinstance(snapshot, messages.StepControlSnapshot):
        step_control.set_pause_state(snapshot.pause_state)
        step_control.set_speed(snapshot.speed)
        step_control.set_noise_parameters(
            snapshot.noise_scale, snapshot.noise_rate
        )
      elif isinstance(snapshot, messages.MjOptionSnapshot):
        if model is not None:
          for field in model.opt._all_fields:  # pylint: disable=protected-access
            val = getattr(snapshot.opt, field)
            try:
              getattr(model.opt, field)[:] = val
            except (TypeError, AttributeError):
              setattr(model.opt, field, val)

    # Send the simulation state to the viewer process as a snapshot.
    if model is not None:
      assert data is not None
      integration_size = mujoco.mj_stateSize(model, integration_sig)
      integration_state = np.empty(integration_size, np.float64)
      mujoco.mj_getState(
          model,
          data,
          integration_state,
          integration_sig,
      )
      self._sim_endpoint.send_to_viewer(
          messages.StateSnapshot(
              state=integration_state, state_sig=integration_sig
          ),
      )

    return model, data, step_control
