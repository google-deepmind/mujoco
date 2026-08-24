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

from typing import Any

from mujoco.experimental.studio import endpoints
from mujoco.experimental.studio import launch_thread
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


def launch_native(
    config: viewer_protocol.ViewerConfig | None = None,
    *,
    viewer_plugins: list[Any] | None = None,
    sim_plugins: list[Any] | None = None,
) -> viewer_handle.ViewerHandle:
  """Launches the NativeViewer in a daemon thread without blocking.

  Args:
    config: Optional ViewerConfig (defaults to standard ViewerConfig()).
    viewer_plugins: Optional list of viewer-side plugin instances.
    sim_plugins: Optional list of sim-side plugin instances.

  Returns:
    A ViewerHandle for interacting with the viewer.
  """
  if config is None:
    config = viewer_protocol.ViewerConfig()

  def target(endpoint: endpoints.ViewerEndpoint) -> None:
    run_native_viewer(config, endpoint, plugins=viewer_plugins)

  return launch_thread.launch_thread(target, sim_plugins=sim_plugins)
