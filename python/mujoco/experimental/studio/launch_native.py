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
"""Native desktop viewer launcher."""

from collections.abc import Iterable
import sys
import threading
from typing import Any, Callable

import mujoco
from mujoco.experimental.studio import endpoints
from mujoco.experimental.studio import launch_thread
from mujoco.experimental.studio import messages
from mujoco.experimental.studio import plugin
from mujoco.experimental.studio import viewer_handle
from mujoco.experimental.studio import viewer_protocol


def run_native_viewer(
    config: viewer_protocol.ViewerConfig,
    endpoint: endpoints.ViewerEndpoint,
    plugins: list[Any] | None = None,
) -> None:
  """Creates a NativeViewer and runs the viewer loop.

  Args:
    config: Configuration specifying the viewer window settings.
    endpoint: Endpoint for communicating with the simulation side.
    plugins: Optional list of viewer-side plugin instances.
  """
  from mujoco.experimental.studio import native_viewer  # pylint: disable=g-import-not-at-top

  viewer = native_viewer.NativeViewer(config, endpoint, plugins=plugins)
  viewer_protocol.run_viewer_loop(viewer)


def _run_viewer_on_main_thread(
    config: viewer_protocol.ViewerConfig,
    sim_loop: Callable[[viewer_handle.ViewerHandle], None],
    *,
    viewer_plugins: list[Any] | None = None,
    sim_plugins: list[Any] | None = None,
) -> None:
  """Runs the native viewer on the calling thread and the sim on a worker.

  This inverts the threading used by ``launch``: the blocking viewer loop stays
  on the calling thread while the simulation runs on a daemon worker thread. It
  is the only arrangement that works on macOS, where AppKit services UI objects
  (such as the ``NSWindow``) only on the process main thread, so creating the
  window on a worker thread fails (see
  https://github.com/google-deepmind/mujoco/issues/3579). It is not
  macOS-specific and works on every platform, which is why ``run`` uses it
  there.

  Args:
    config: Viewer window configuration.
    sim_loop: Called on the worker thread with the sim-side ``ViewerHandle``; it
      publishes a model and drives the sync loop (e.g. ``run_sim_loop``).
    viewer_plugins: Optional viewer-side plugin instances.
    sim_plugins: Optional sim-side plugin instances.
  """
  viewer_endpoint, sim_endpoint = launch_thread.make_thread_endpoints()

  # The viewer owns the calling thread, so the handle has no viewer thread to
  # join; ``stop`` lets the worker's ``is_running()`` notice once the viewer
  # loop has returned, in case the viewer died without sending an ExitEvent.
  stop = threading.Event()
  handle = viewer_handle.ViewerHandle(
      sim_endpoint,
      sim_plugins=sim_plugins,
      is_alive_fn=lambda: not stop.is_set(),
      shutdown_fn=lambda _timeout: None,
  )

  # An exception escaping the sim loop would otherwise only reach the threading
  # excepthook; keep it so it can be re-raised on the calling thread, the way
  # launch_thread forwards a viewer-thread exception to the sim side.
  error: BaseException | None = None

  def run_sim() -> None:
    nonlocal error
    try:
      # close() on exit sends an ExitEvent that stops the viewer loop below.
      with handle:
        sim_loop(handle)
    except BaseException as e:  # pylint: disable=broad-exception-caught
      error = e

  sim_thread = threading.Thread(target=run_sim, daemon=True)
  sim_thread.start()
  try:
    # Blocks until the window closes or the sim side requests an exit.
    run_native_viewer(config, viewer_endpoint, plugins=viewer_plugins)
  finally:
    stop.set()
    sim_thread.join(timeout=5.0)
  if error is not None:
    raise error


def launch(
    config: viewer_protocol.ViewerConfig | None = None,
    *,
    plugins: Iterable[plugin.Plugin] | None = None,
) -> viewer_handle.ViewerHandle:
  """Launches the NativeViewer in a daemon thread without blocking.

  Args:
    config: Optional ViewerConfig (defaults to standard ViewerConfig()).
    plugins: Optional viewer and sim plugins.

  Returns:
    A ViewerHandle for interacting with the viewer.

  Raises:
    RuntimeError: On macOS, where the native window must be created on the main
      thread and so cannot be launched on a worker thread; use ``run`` instead.
  """
  if config is None:
    config = viewer_protocol.ViewerConfig()
  if sys.platform == 'darwin':
    # AppKit only services UI objects created on the process main thread, so
    # the viewer-on-a-worker-thread arrangement here cannot create its window
    # on macOS. Fail loudly and point at run(), which keeps the viewer on the
    # calling thread, rather than letting the window silently fail to open.
    raise RuntimeError(
        'launch() cannot start the native Studio viewer on macOS: the native '
        'window must be created on the process main thread, but launch() runs '
        'the viewer on a worker thread (see '
        'https://github.com/google-deepmind/mujoco/issues/3579). Use run(), '
        'which keeps the viewer on the calling thread and runs the simulation '
        'on a worker thread.'
    )
  viewer_plugins, sim_plugins = plugin.split_plugins(plugins)

  def target(endpoint: endpoints.ViewerEndpoint) -> None:
    run_native_viewer(config, endpoint, plugins=viewer_plugins)

  return launch_thread.launch_thread(target, sim_plugins=sim_plugins)


def run(
    config: viewer_protocol.ViewerConfig | None = None,
    *,
    model: mujoco.MjModel | None = None,
    data: mujoco.MjData | None = None,
    model_path: str | None = None,
    plugins: Iterable[plugin.Plugin] | None = None,
) -> None:
  """Launches the NativeViewer and runs the blocking simulation sync loop.

  Unlike ``launch``, this works on macOS: it keeps the viewer on the calling
  thread (as AppKit requires) and runs the simulation on a worker thread.

  Args:
    config: Optional ViewerConfig (defaults to standard ViewerConfig()).
    model: In-memory model to publish instead.
    data: The data for ``model``; a fresh one is created when None.
    model_path: Model file to load.
    plugins: Optional viewer and sim plugins.

  Raises:
    ValueError: If both ``model`` and ``model_path`` are given.
  """
  if model is not None and model_path:
    raise ValueError('Pass either model_path or model, not both.')
  if config is None:
    config = viewer_protocol.ViewerConfig()

  def sim_loop(handle: viewer_handle.ViewerHandle) -> None:
    if model is not None:
      handle.set_model(model, data=data)
    elif model_path:
      handle.dispatch(messages.LoadModelEvent(path=model_path))
    viewer_handle.run_sim_loop(handle)

  if sys.platform == 'darwin':
    # macOS requires the native window on the main thread, so run the viewer on
    # the calling thread and the simulation on a worker thread.
    viewer_plugins, sim_plugins = plugin.split_plugins(plugins)
    _run_viewer_on_main_thread(
        config,
        sim_loop,
        viewer_plugins=viewer_plugins,
        sim_plugins=sim_plugins,
    )
  else:
    # Preserve the established arrangement on Linux and Windows: the viewer on a
    # worker thread, the simulation sync loop on the calling thread.
    with launch(config, plugins=plugins) as handle:
      sim_loop(handle)
