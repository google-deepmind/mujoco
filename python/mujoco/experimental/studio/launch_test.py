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
"""Tests for launch modules."""

import threading
from unittest import mock
from absl.testing import absltest
from mujoco.experimental.studio import endpoints
from mujoco.experimental.studio import launch_thread
from mujoco.experimental.studio import launch_web
from mujoco.experimental.studio import messages
from mujoco.experimental.studio import viewer_protocol
import numpy as np


class LaunchTest(absltest.TestCase):

  def test_thread_snapshot_channel(self):
    channel = launch_thread.ThreadSnapshotChannel()
    s1 = messages.StateSnapshot(state=np.array([1.0, 2.0]), state_sig=1)
    s2 = messages.StateSnapshot(state=np.array([3.0, 4.0]), state_sig=1)

    channel.put(s1)
    channel.put(s2)  # Should overwrite s1 (latest-wins)

    snapshots = channel.get()
    self.assertLen(snapshots, 1)
    self.assertEqual(snapshots[0], s2)

    # Subsequent get should be empty
    self.assertEmpty(channel.get())

  def test_thread_event_channel(self):
    channel = launch_thread.ThreadEventChannel()
    e1 = messages.ResetEvent()
    e2 = messages.ExitEvent()

    channel.put(e1)
    channel.put(e2)

    events = channel.get()
    self.assertEqual(events, [e1, e2])
    self.assertEmpty(channel.get())

  def test_make_thread_endpoints(self):
    viewer_ep, sim_ep = launch_thread.make_thread_endpoints()
    self.assertIsInstance(viewer_ep, endpoints.ViewerEndpoint)
    self.assertIsInstance(sim_ep, endpoints.SimEndpoint)

    # Sim -> Viewer communication
    snapshot = messages.StateSnapshot(state=np.array([1.0]), state_sig=1)
    event = messages.ResetEvent()
    sim_ep.send_to_viewer(snapshot)
    sim_ep.send_to_viewer(event)

    self.assertEqual(viewer_ep.get_sim_snapshots(), [snapshot])
    self.assertEqual(viewer_ep.get_sim_events(), [event])

    # Viewer -> Sim communication
    v_event = messages.ExitEvent()
    viewer_ep.send_to_sim(v_event)
    self.assertEqual(sim_ep.get_viewer_events(), [v_event])

  def test_launch_thread(self):
    stop_event = threading.Event()
    started_event = threading.Event()

    def mock_target(endpoint: endpoints.ViewerEndpoint):
      del endpoint
      started_event.set()
      stop_event.wait()

    handle = launch_thread.launch_thread(mock_target)
    started_event.wait(timeout=5.0)
    self.assertTrue(handle.is_running())
    stop_event.set()
    handle.close()
    self.assertFalse(handle.is_running())

  @mock.patch.object(launch_web, 'run_web_viewer')
  def test_launch_web(self, mock_run_web):
    stop_event = threading.Event()
    started_event = threading.Event()

    def fake_run_web(config, endpoint, **kwargs):
      del config, endpoint, kwargs
      started_event.set()
      stop_event.wait()

    mock_run_web.side_effect = fake_run_web
    cfg_web = viewer_protocol.ViewerConfig(gfx='web')
    handle = launch_web.launch_web(cfg_web)
    started_event.wait(timeout=5.0)
    self.assertTrue(handle.is_running())
    stop_event.set()
    handle.close()


if __name__ == '__main__':
  absltest.main()
