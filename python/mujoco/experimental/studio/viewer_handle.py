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
"""Viewer handle and event handlers for the simulation side."""

from typing import Any, Callable
import mujoco
from mujoco.experimental.studio import endpoints
from mujoco.experimental.studio import handler_registry
from mujoco.experimental.studio import messages
from mujoco.experimental.studio import sim as _sim
import numpy as np


class ViewerHandle:
  """A handle for interacting with a running Studio application from the sim."""

  def __init__(
      self,
      sim_endpoint: endpoints.SimEndpoint,
      *,
      handlers: list[Any] | None = None,
      is_alive_fn: Callable[[], bool] | None = None,
  ) -> None:
    """Initializes the ViewerHandle.

    Args:
      sim_endpoint: The endpoint to use for communication with the viewer.
      handlers: Optional list of handler instances for sim-side processing,
        which are classes with methods decorated with ``@handler``.
      is_alive_fn: Optional function called to check if the viewer is still
        alive/responsive. If not provided, the viewer is assumed to be running
        until ``close()`` is called.
    """

    self._sim_endpoint = sim_endpoint
    self._is_running = True
    self._is_alive_fn = is_alive_fn
    self.model: mujoco.MjModel | None = None
    self.data: mujoco.MjData | None = None
    self.step_control: _sim.StepControl | None = None

    # Instantiate handlers from user handlers + framework defaults.
    all_handlers: list[Any] = list(handlers or [])
    all_handlers.append(self)
    self._handlers = handler_registry.HandlerRegistry(all_handlers)

  def close(self) -> None:
    """Signals the viewer to exit and closes the sim endpoint."""
    if self._is_running:
      self._is_running = False
      try:
        self.send_to_viewer(messages.ExitEvent())
      except Exception:  # pylint: disable=broad-exception-caught
        pass  # Ignore exceptions, the viewer may have already closed.
    self._sim_endpoint.close()

  def __enter__(self) -> 'ViewerHandle':
    return self

  def __exit__(
      self,
      exc_type: type[BaseException] | None,
      exc_val: BaseException | None,
      exc_tb: Any,
  ) -> None:
    self.close()

  def is_running(self) -> bool:
    """Returns True while the viewer is open."""
    if self._is_alive_fn is not None and not self._is_alive_fn():
      self.close()
    return self._is_running

  def send_to_viewer(self, message: messages.Message) -> None:
    """Sends an event or snapshot message to the viewer process.

    Args:
      message: The message to send.
    """
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

    self.model, self.data, self.step_control = model, data, step_control

    # Process incoming events from the viewer.
    for event in self._sim_endpoint.get_viewer_events():
      self._handlers.dispatch(event)

    # Process incoming snapshots from the viewer.
    for snapshot in self._sim_endpoint.get_viewer_snapshots():
      self._handlers.dispatch(snapshot)

    model, data, step_control = self.model, self.data, self.step_control

    # Send the simulation state to the viewer process as a snapshot.
    if model is not None:
      assert data is not None
      integration_sig = int(mujoco.mjtState.mjSTATE_INTEGRATION)
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

  @messages.handler(priority=messages.Priority.INTERNAL)
  def _on_model(self, event: messages.ModelEvent) -> bool:
    self.model = event.model
    self.data = mujoco.MjData(event.model)
    mujoco.mj_forward(self.model, self.data)
    self.step_control = _sim.StepControl()
    return True

  @messages.handler(priority=messages.Priority.INTERNAL)
  def _on_perturb(self, event: messages.PerturbEvent) -> bool:
    model = self.model
    data = self.data
    if model is not None and data is not None:
      state_size = mujoco.mj_stateSize(model, event.state_sig)
      if len(event.state) == state_size:
        mujoco.mj_setState(model, data, event.state, event.state_sig)
    return True

  @messages.handler(priority=messages.Priority.INTERNAL)
  def _on_reset(self, _: messages.ResetEvent) -> bool:
    model = self.model
    data = self.data
    if model is not None:
      assert data is not None
      mujoco.mj_resetData(model, data)
      mujoco.mj_forward(model, data)
    return True

  @messages.handler(priority=messages.Priority.INTERNAL)
  def _on_exit(self, _: messages.ExitEvent) -> bool:
    self._is_running = False  # pylint: disable=protected-access
    return True

  @messages.handler(priority=messages.Priority.INTERNAL)
  def _on_step_control(self, event: messages.StepControlSnapshot) -> bool:
    sc = self.step_control
    if sc is not None:
      sc.set_pause_state(event.pause_state)
      sc.set_speed(event.speed)
      sc.set_noise_parameters(event.noise_scale, event.noise_rate)
    return True

  @messages.handler(priority=messages.Priority.INTERNAL)
  def _on_mjoption(self, event: messages.MjOptionSnapshot) -> bool:
    model = self.model
    if model is not None:
      for field in model.opt._all_fields:  # pylint: disable=protected-access
        val = getattr(event.opt, field)
        try:
          getattr(model.opt, field)[:] = val
        except (TypeError, AttributeError):
          setattr(model.opt, field, val)
    return True
