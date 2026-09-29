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

import dataclasses
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


@dataclasses.dataclass(frozen=True)
class SimInitEvent(messages.Event):
  """Lifecycle event dispatched once when the ViewerHandle is initialized.

  The sim-side counterpart of ``viewer_protocol.ViewerInitEvent``: plugins that
  need the handle, to publish a model or to send messages to the viewer, should
  handle this event and cache the reference rather than having the launcher
  hand it to them.

  Like ``ViewerInitEvent``, this carries a live object, so it is dispatched
  locally to ``sim_plugins`` and never crosses the sim/viewer endpoint.

  Dispatched from ``ViewerHandle.__init__``, so a handler runs before
  ``launch_passive`` (or any other launcher) has returned. The outbound endpoint
  queues are already initialized, so handlers may cache the handle and enqueue
  initial messages via ``event.handle.send_to_viewer(...)``. Raising from a
  handler aborts the launch: the viewer is shut down and the exception
  propagates out of the launcher.
  """

  handle: 'ViewerHandle'


class ViewerHandle:
  """A handle for interacting with a running viewer application from the sim."""

  def __init__(
      self,
      sim_endpoint: endpoints.SimEndpoint,
      *,
      sim_plugins: list[Any] | None = None,
      is_alive_fn: IsAliveFn | None = None,
      shutdown_fn: ShutdownFn | None = None,
  ) -> None:
    """Initializes the ViewerHandle.

    Args:
      sim_endpoint: The endpoint to use for communication with the viewer.
      sim_plugins: Optional list of plugin instances for sim-side processing,
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
    all_sim_plugins: list[Any] = list(sim_plugins or [])
    all_sim_plugins.append(self)
    self._sim_plugins = plugin_registry.PluginRegistry(all_sim_plugins)

    # Hand the plugins a reference to this handle. Dispatched last, so that a
    # plugin's handler sees a fully constructed handle.
    try:
      self._sim_plugins.dispatch(SimInitEvent(handle=self))
    except BaseException:  # pylint: disable=broad-exception-caught
      # Launchers start the viewer before they construct the handle, so one is
      # already running. Raising out of __init__ means the caller never gets a
      # handle and never enters the ``with`` block, so nothing else will ever
      # call close(): shut the viewer down here or it is left running with no
      # way to stop it.
      self._is_running = False
      self.close()
      raise

  def close(self) -> None:
    """Signals the viewer to exit and waits for it to shut down."""
    if self._is_running:
      self._is_running = False
      self.dispatch(messages.ExitEvent())
    try:
      self.send_to_viewer(messages.ExitEvent())
    except Exception:  # pylint: disable=broad-exception-caught
      pass  # Ignore exceptions, the viewer may have already closed.
    if self._shutdown_fn is not None:
      self._shutdown_fn(5.0)
      self._shutdown_fn = None
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

  def dispatch(self, message: messages.Message) -> None:
    """Dispatches a message to registered sim-side handlers in priority order."""
    self._sim_plugins.dispatch(message)

  def set_model(self, model: mujoco.MjModel, path: str = '') -> None:
    """Replaces the simulation's model and notifies both sim plugins and the viewer."""
    event = messages.ModelEvent(model=model, path=path)
    self.dispatch(event)
    self.send_to_viewer(event)

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

    if model is not None and model is not self.model:
      self._sim_endpoint.send_to_viewer(messages.ModelEvent(model=model))
    self.model, self.data = model, data

    # Process incoming events from the viewer.
    for event in self._sim_endpoint.get_viewer_events():
      self._sim_plugins.dispatch(event)

    # Process incoming snapshots from the viewer.
    for snapshot in self._sim_endpoint.get_viewer_snapshots():
      self._sim_plugins.dispatch(snapshot)

    if self.model is not None:
      assert self.data is not None
      # Advance the simulation: dispatched locally to sim-side plugins.
      self._sim_plugins.dispatch(
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
  def _on_reset(self, event: messages.ResetEvent) -> bool:
    model = self.model
    data = self.data
    if model is not None:
      assert data is not None
      if event.key is not None:
        mujoco.mj_resetDataKeyframe(model, data, event.key)
      else:
        mujoco.mj_resetData(model, data)
      mujoco.mj_forward(model, data)
    return True

  @messages.handler(priority=messages.Priority.INTERNAL)
  def _on_exit(self, _: messages.ExitEvent) -> None:
    self._is_running = False  # pylint: disable=protected-access

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


def run_sim_loop(
    handle: ViewerHandle,
    model: mujoco.MjModel | None = None,
    data: mujoco.MjData | None = None,
    model_path: str | None = None,
) -> None:
  """Runs the standard simulation sync loop until the viewer closes."""
  if model is not None:
    handle.set_model(model, path=model_path or '')
  try:
    while handle.is_running():
      model, data = handle.sync(model, data)
  except KeyboardInterrupt:
    # Ctrl+C is the documented way to quit; exit cleanly, no traceback.
    print('\nShutting down.', flush=True)
