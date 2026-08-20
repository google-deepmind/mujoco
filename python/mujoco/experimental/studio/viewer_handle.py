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
from mujoco.experimental.studio import messages
from mujoco.experimental.studio import plugin_registry
import numpy as np

# Launcher-owned liveness check: returns True while the viewer is still alive.
# This enables the handle to notice a viewer died without sending ExitEvent.
IsAliveFn = Callable[[], bool]

# Launcher-owned shutdown function: waits up to the given timeout (seconds) for
# the viewer to finish after close() has sent the ExitEvent. Waiting lets the
# viewer release its resources before the interpreter tears itself down.
ShutdownFn = Callable[[float], None]


class ViewerHandle:
  """A handle for interacting with a running viewer application from the sim."""

  def __init__(
      self,
      sim_endpoint: endpoints.SimEndpoint,
      *,
      plugins: list[Any] | None = None,
      is_alive_fn: IsAliveFn | None = None,
      shutdown_fn: ShutdownFn | None = None,
  ) -> None:
    """Initializes the ViewerHandle.

    Args:
      sim_endpoint: The endpoint to use for communication with the viewer.
      plugins: Optional list of plugin instances for sim-side processing, which
        are classes with methods decorated with ``@handler``.
      is_alive_fn: Optional liveness check; without one the viewer is assumed to
        be running until ``close()`` is called.
      shutdown_fn: Optional launcher-owned shutdown hook, called by ``close()``.
    """

    self._sim_endpoint = sim_endpoint
    self._is_running = True
    self._is_alive_fn = is_alive_fn
    self._shutdown_fn = shutdown_fn
    self.model: mujoco.MjModel | None = None
    self.data: mujoco.MjData | None = None

    # Instantiate handlers from user plugins + framework defaults.
    all_plugins: list[Any] = list(plugins or [])
    all_plugins.append(self)
    self._plugins = plugin_registry.PluginRegistry(all_plugins)

  def close(self) -> None:
    """Signals the viewer to exit and waits for it to shut down."""
    if self._is_running:
      self._is_running = False
      try:
        self.send_to_viewer(messages.ExitEvent())
      except Exception:  # pylint: disable=broad-exception-caught
        pass  # Ignore exceptions, the viewer may have already closed.
      if self._shutdown_fn is not None:
        self._shutdown_fn(5.0)
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
  ) -> tuple[mujoco.MjModel | None, mujoco.MjData | None]:
    """Syncs the simulation with the viewer and returns the updated sim state.

    This method processes incoming messages from the viewer, dispatches a
    ``StepEvent`` so sim-side plugins can advance the simulation, and sends the
    resulting simulation state to the viewer as a snapshot.

    Stepping and pacing are plugin responsibilities: pass
    ``step_control.StepControl()`` in ``sim_plugins`` for the standard
    real-time-paced CPU stepping, or your own plugin to step differently.
    Without a stepping plugin nothing advances and ``sync`` never sleeps, so
    the calling loop must pace itself to avoid busy-spinning.

    Args:
      model: The current model.
      data: The current data.

    Returns:
      The updated model and data; rebind both, they may be new objects (e.g.
      after the viewer sends a ModelEvent).
    """

    self.model, self.data = model, data

    # Process incoming events from the viewer.
    for event in self._sim_endpoint.get_viewer_events():
      self._plugins.dispatch(event)

    # Process incoming snapshots from the viewer.
    for snapshot in self._sim_endpoint.get_viewer_snapshots():
      self._plugins.dispatch(snapshot)

    if self.model is not None:
      assert self.data is not None
      # Advance the simulation: dispatched locally to sim-side plugins.
      self._plugins.dispatch(
          messages.StepEvent(model=self.model, data=self.data)
      )

      # Send the simulation state to the viewer process as a snapshot.
      integration_sig = int(mujoco.mjtState.mjSTATE_INTEGRATION)
      integration_size = mujoco.mj_stateSize(self.model, integration_sig)
      integration_state = np.empty(integration_size, np.float64)
      mujoco.mj_getState(
          self.model,
          self.data,
          integration_state,
          integration_sig,
      )
      self._sim_endpoint.send_to_viewer(
          messages.StateSnapshot(
              state=integration_state, state_sig=integration_sig
          ),
      )

    return self.model, self.data

  @messages.handler(priority=messages.Priority.INTERNAL)
  def _on_model(self, event: messages.ModelEvent) -> bool:
    self.model = event.model
    self.data = mujoco.MjData(event.model)
    mujoco.mj_forward(self.model, self.data)
    return True

  @messages.handler(priority=messages.Priority.INTERNAL)
  def _on_state(self, event: messages.StateEvent) -> bool:
    model = self.model
    data = self.data
    if model is not None and data is not None:
      state_size = mujoco.mj_stateSize(model, event.state_sig)
      if len(event.state) == state_size:
        mujoco.mj_setState(model, data, event.state, event.state_sig)
        mujoco.mj_forward(model, data)
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
