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


class _CachingPlugin:
  """The normal case: a plugin that caches the handle it is handed."""

  def __init__(self) -> None:
    self.handle: viewer_handle.ViewerHandle | None = None

  @messages.handler
  def on_sim_init(self, event: viewer_handle.SimInitEvent) -> None:
    self.handle = event.handle


class _RaisingPlugin:
  """A plugin whose handler fails during handle construction."""

  @messages.handler
  def on_sim_init(self, event: viewer_handle.SimInitEvent) -> None:
    del event
    raise ValueError('handler failed')


class ViewerHandleTest(absltest.TestCase):

  def test_sim_init_event_hands_the_handle_to_plugins(self):
    _, sim_endpoint = launch_thread.make_thread_endpoints()
    plugin = _CachingPlugin()

    handle = viewer_handle.ViewerHandle(sim_endpoint, sim_plugins=[plugin])

    self.assertIs(plugin.handle, handle)

  def test_raising_handler_shuts_the_viewer_down(self):
    # A launcher starts the viewer before it builds the handle, so a handler
    # that raises must not escape without stopping the viewer first: nobody
    # else can, as the caller never receives a handle to close.
    _, sim_endpoint = launch_thread.make_thread_endpoints()
    shutdown_timeouts = []

    with self.assertRaises(ValueError):
      viewer_handle.ViewerHandle(
          sim_endpoint,
          sim_plugins=[_RaisingPlugin()],
          shutdown_fn=shutdown_timeouts.append,
      )

    self.assertLen(shutdown_timeouts, 1)

  def test_sync_sends_model_event_on_model_change(self):
    viewer_endpoint, sim_endpoint = launch_thread.make_thread_endpoints()
    handle = viewer_handle.ViewerHandle(sim_endpoint)
    model = mujoco.MjModel.from_xml_string('<mujoco/>')
    data = mujoco.MjData(model)

    # First sync with a model sends a ModelEvent to the viewer.
    handle.sync(model, data)
    events = viewer_endpoint.get_sim_events()
    self.assertLen(events, 1)
    self.assertIsInstance(events[0], messages.ModelEvent)
    self.assertIs(events[0].model, model)

    # Subsequent sync with the same model does not resend ModelEvent.
    handle.sync(model, data)
    self.assertEmpty(viewer_endpoint.get_sim_events())

  def test_run_sim_loop_preserves_model_path_without_duplicate_model_event(
      self,
  ):
    viewer_endpoint, sim_endpoint = launch_thread.make_thread_endpoints()
    handle = viewer_handle.ViewerHandle(sim_endpoint)
    model = mujoco.MjModel.from_xml_string('<mujoco/>')
    data = mujoco.MjData(model)

    # Queue an ExitEvent from the viewer so run_sim_loop terminates after one
    # sync iteration.
    viewer_endpoint.send_to_sim(messages.ExitEvent())

    viewer_handle.run_sim_loop(
        handle, model=model, data=data, model_path='/path/to/model.xml'
    )

    events = viewer_endpoint.get_sim_events()
    model_events = [e for e in events if isinstance(e, messages.ModelEvent)]
    self.assertLen(model_events, 1)
    self.assertEqual(model_events[0].path, '/path/to/model.xml')

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
