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

from mujoco.experimental.studio import endpoints
from mujoco.experimental.studio import messages
from mujoco.experimental.studio import sim_app
from mujoco.experimental.studio import viewer_app
from mujoco.experimental.studio import viewer_protocol

sa = sim_app
va = viewer_app
vp = viewer_protocol


class _PassiveSnapshotChannel:
  """SnapshotChannel for passive mode: per-type snapshot by reference, wakes the viewer thread on put."""

  def __init__(self):
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
  """EventChannel for passive mode: a thread-safe queue.Queue of event messages."""

  def __init__(self):
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


def _run_viewer_target(
    config: viewer_protocol.ViewerConfig,
    viewer_endpoint: endpoints.ViewerEndpoint,
    viewer_gui_hook: va.ViewerGuiHook | None = None,
    viewer_update_hook: va.ViewerUpdateHook | None = None,
    viewer_event_handler: va.ViewerEventHandler | None = None,
    viewer_snapshot_handler: va.ViewerSnapshotHandler | None = None,
) -> None:
  """Creates the appropriate viewer and runs the viewer loop."""

  if config.viewer_mode == viewer_protocol.ViewerMode.NATIVE:
    from mujoco.experimental.studio import native_viewer  # pylint: disable=g-import-not-at-top

    viewer = native_viewer.NativeViewer(config)
  elif config.viewer_mode == viewer_protocol.ViewerMode.WEB:
    raise NotImplementedError('Web viewer not implemented yet')
  else:
    raise ValueError(f'Unknown viewer mode: {config.viewer_mode!r}')

  va.run_viewer(
      viewer,
      viewer_endpoint,
      viewer_gui_hook=viewer_gui_hook,
      viewer_update_hook=viewer_update_hook,
      viewer_event_handler=viewer_event_handler,
      viewer_snapshot_handler=viewer_snapshot_handler,
  )


def launch_passive(
    config: vp.ViewerConfig,
    *,
    viewer_gui_hook: va.ViewerGuiHook | None = None,
    viewer_update_hook: va.ViewerUpdateHook | None = None,
    viewer_event_handler: va.ViewerEventHandler | None = None,
    viewer_snapshot_handler: va.ViewerSnapshotHandler | None = None,
    sim_event_handler: sa.SimEventHandler | None = None,
) -> sa.ViewerHandle:
  """Launches the Studio GUI in a daemon thread without blocking.

  The viewer runs the full Studio UI (toolbar, options, inspector) on the
  rendering thread.  The caller keeps running and pushes state via
  ``handle.sync()``.

  Args:
    config: Viewer window configuration.
    viewer_gui_hook: Optional hook to draw custom ImGui panels.
    viewer_update_hook: Optional hook called once per frame before GUI.
    viewer_event_handler: Optional handler for sim-to-viewer events.
    viewer_snapshot_handler: Optional handler for sim-to-viewer snapshots.
    sim_event_handler: Optional handler for viewer-to-sim events.

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
      target=_run_viewer_target,
      args=(
          config,
          viewer_endpoint,
          viewer_gui_hook,
          viewer_update_hook,
          viewer_event_handler,
          viewer_snapshot_handler,
      ),
      daemon=True,
  )
  thread.start()

  handle = sa.ViewerHandle(
      sim_endpoint,
      is_alive_fn=thread.is_alive,
      sim_event_handler=sim_event_handler,
  )
  return handle
