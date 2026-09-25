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
"""Tests for the sim-side ViewerHandle."""

from absl.testing import absltest
import mujoco
from mujoco.experimental.studio import launch_thread
from mujoco.experimental.studio import messages
from mujoco.experimental.studio import viewer_handle


class ViewerHandleTest(absltest.TestCase):

  def test_close_dispatches_exit_event_once_and_runs_shutdown_after_viewer_exit(
      self,
  ):
    viewer_endpoint, sim_endpoint = launch_thread.make_thread_endpoints()
    exit_events = []
    shutdown_timeouts = []

    class _ExitPlugin:

      @messages.handler
      def on_exit(self, event: messages.ExitEvent) -> None:
        exit_events.append(event)

    handle = viewer_handle.ViewerHandle(
        sim_endpoint,
        sim_plugins=[_ExitPlugin()],
        shutdown_fn=shutdown_timeouts.append,
    )
    model = mujoco.MjModel.from_xml_string('<mujoco/>')
    data = mujoco.MjData(model)

    # Viewer initiates shutdown by sending ExitEvent during sync().
    viewer_endpoint.send_to_sim(messages.ExitEvent())
    handle.sync(model, data)
    self.assertFalse(handle.is_running())
    self.assertLen(exit_events, 1)

    # Subsequent close() still invokes shutdown_fn and does not dispatch a
    # duplicate ExitEvent.
    handle.close()
    self.assertLen(shutdown_timeouts, 1)
    self.assertLen(exit_events, 1)


if __name__ == '__main__':
  absltest.main()
