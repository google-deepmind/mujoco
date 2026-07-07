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
"""Non-blocking Studio launcher.

Launches the full Studio GUI in a daemon thread and returns a ``ViewerHandle``
that the calling thread uses to push simulation state into the viewer.
"""

import queue
import threading
from typing import Any

from mujoco.experimental.studio import endpoints
from mujoco.experimental.studio import messages
from mujoco.experimental.studio import viewer_handle
from mujoco.experimental.studio import viewer_protocol


class _PassiveSnapshotChannel(messages.SnapshotChannel):
  """SnapshotChannel for passive mode.

  Stores per-type snapshot by reference and wakes the viewer thread when put.
  """

  def __init__(self) -> None:
    self._pending_snapshots: dict[
        type[messages.Snapshot], messages.Snapshot
    ] = {}
    self._lock = threading.Lock()
    self._wakeup = threading.Event()

  def put(self, snapshot: messages.Snapshot) -> None:
    with self._lock:
      self._pending_snapshots[type(snapshot)] = snapshot
    self._wakeup.set()

  def get(self) -> list[messages.Snapshot]:
    self._wakeup.clear()
    with self._lock:
      out = list(self._pending_snapshots.values())
      self._pending_snapshots.clear()
      return out

  def close(self) -> None:
    pass


class _PassiveEventChannel(messages.EventChannel):
  """EventChannel for passive mode using a thread-safe queue."""

  def __init__(self) -> None:
    self._events: queue.Queue[messages.Event] = queue.Queue()

  def put(self, event: messages.Event) -> None:
    self._events.put(event)

  def get(self) -> list[messages.Event]:
    out = []
    while True:
      try:
        out.append(self._events.get_nowait())
      except queue.Empty:
        break
    return out

  def close(self) -> None:
    pass


def run_viewer_target(
    config: viewer_protocol.ViewerConfig,
    viewer_endpoint: endpoints.ViewerEndpoint,
    handlers: list[Any] | None = None,
) -> None:
  """Creates the appropriate viewer and runs the viewer loop.

  Args:
    config: Configuration specifying the viewer mode and window settings.
    viewer_endpoint: Endpoint for communicating with the simulation side.
    handlers: Optional list of viewer-side handler instances, which are classes
      with methods decorated with ``@handler``.

  Raises:
    ValueError: If the viewer mode requested in config is unknown.
    NotImplementedError: If web mode is requested.
  """

  if config.viewer_mode == viewer_protocol.ViewerMode.NATIVE:
    from mujoco.experimental.studio import native_viewer  # pylint: disable=g-import-not-at-top

    viewer = native_viewer.NativeViewer(
        config, viewer_endpoint, handlers=handlers
    )
  elif config.viewer_mode == viewer_protocol.ViewerMode.WEB:
    raise NotImplementedError('Web viewer not implemented yet')
  else:
    raise ValueError(f'Unknown viewer mode: {config.viewer_mode!r}')

  viewer_protocol.run_viewer_loop(viewer)


def launch_passive(
    config: viewer_protocol.ViewerConfig,
    *,
    viewer_handlers: list[Any] | None = None,
    sim_handlers: list[Any] | None = None,
) -> viewer_handle.ViewerHandle:
  """Launches the Studio GUI in a daemon thread without blocking.

  The viewer runs the full Studio UI (toolbar, options, inspector) on the
  rendering thread.  The caller keeps running and pushes state via
  ``handle.sync()``.

  Args:
    config: Viewer window configuration.
    viewer_handlers: Optional list of viewer-side handler instances, which are
      classes with methods decorated with ``@handler``.
    sim_handlers: Optional list of sim-side handler instances, which are classes
      with methods decorated with ``@handler``.

  Returns:
    A ViewerHandle for interacting with the viewer.
  """
  viewer_endpoint, sim_endpoint = endpoints.make_endpoints(
      s2v_snapshot=_PassiveSnapshotChannel(),
      s2v_events=_PassiveEventChannel(),
      v2s_events=_PassiveEventChannel(),
      v2s_snapshot=_PassiveSnapshotChannel(),
  )

  thread = threading.Thread(
      target=run_viewer_target,
      args=(config, viewer_endpoint, viewer_handlers),
      daemon=True,
  )
  thread.start()

  handle = viewer_handle.ViewerHandle(
      sim_endpoint,
      is_alive_fn=thread.is_alive,
      handlers=sim_handlers,
  )
  return handle
