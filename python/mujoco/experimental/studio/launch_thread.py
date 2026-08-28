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
"""In-process threaded viewer launcher functionality and channels."""

import queue
import threading
from typing import Any, Callable

from mujoco.experimental.studio import endpoints
from mujoco.experimental.studio import messages
from mujoco.experimental.studio import viewer_handle
from mujoco.experimental.studio import viewer_protocol


class ThreadSnapshotChannel(messages.SnapshotChannel):
  """SnapshotChannel for in-process threaded mode.

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


class ThreadEventChannel(messages.EventChannel):
  """EventChannel for in-process threaded mode using a thread-safe queue."""

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


def make_thread_endpoints() -> (
    tuple[endpoints.ViewerEndpoint, endpoints.SimEndpoint]
):
  """Creates connected in-process thread-safe endpoints."""
  return endpoints.make_endpoints(
      s2v_snapshot=ThreadSnapshotChannel(),
      s2v_events=ThreadEventChannel(),
      v2s_events=ThreadEventChannel(),
      v2s_snapshot=ThreadSnapshotChannel(),
  )


def launch_thread(
    target_fn: Callable[[endpoints.ViewerEndpoint], None],
    *,
    sim_plugins: list[Any] | None = None,
) -> viewer_handle.ViewerHandle:
  """Launches a viewer target function in a daemon thread and returns a ViewerHandle."""
  viewer_endpoint, sim_endpoint = make_thread_endpoints()

  thread = threading.Thread(
      target=target_fn,
      args=(viewer_endpoint,),
      daemon=True,
  )
  thread.start()

  handle = viewer_handle.ViewerHandle(
      sim_endpoint,
      is_alive_fn=thread.is_alive,
      shutdown_fn=thread.join,
      plugins=sim_plugins,
  )
  return handle
