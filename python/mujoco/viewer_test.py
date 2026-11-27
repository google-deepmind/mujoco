# Copyright 2022 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
"""Minimal tests of mujoco.viewer that do not open a window."""

import threading
from unittest import mock

from absl.testing import absltest
from mujoco import viewer


class _FakeSim:
  """Stands in for Simulate: `exit` releases the fake render thread."""

  def __init__(self):
    self.exit_requested = threading.Event()

  def exit(self):
    self.exit_requested.set()


def _fake_handle(sim):
  return viewer.Handle(sim, cam=None, opt=None, pert=None, user_scn=None)


class ViewerTest(absltest.TestCase):
  # pylint: disable=protected-access

  def test_launch_function_exists(self):
    self.assertIsNotNone(viewer.launch)

  def test_close_waits_for_render_thread(self):
    sim = _FakeSim()
    finished = threading.Event()

    def render_loop():
      sim.exit_requested.wait()
      finished.set()  # stands in for destroying the window after the loop

    thread = threading.Thread(target=render_loop, daemon=True)
    thread.start()
    handle = _fake_handle(sim)
    handle._thread = thread

    handle.close()
    self.assertTrue(finished.is_set())
    self.assertFalse(thread.is_alive())
    handle.close()  # closing again is harmless

  def test_close_from_render_thread_does_not_join_itself(self):
    sim = _FakeSim()
    handle = _fake_handle(sim)
    closed = threading.Event()

    def render_loop():
      handle._thread = threading.current_thread()
      handle.close()  # e.g. from a key callback
      closed.set()

    thread = threading.Thread(target=render_loop, daemon=True)
    thread.start()
    thread.join(timeout=5)
    self.assertTrue(closed.is_set())
    self.assertTrue(sim.exit_requested.is_set())

  def test_close_wait_is_bounded(self):
    sim = _FakeSim()
    stop = threading.Event()
    thread = threading.Thread(target=stop.wait, daemon=True)  # never exits
    thread.start()
    handle = _fake_handle(sim)
    handle._thread = thread

    with mock.patch.object(viewer, '_CLOSE_TIMEOUT_SECONDS', 0.1):
      handle.close()
    self.assertTrue(sim.exit_requested.is_set())
    self.assertTrue(thread.is_alive())
    stop.set()

  def test_close_after_viewer_is_gone(self):
    sim = _FakeSim()
    handle = _fake_handle(sim)
    del sim
    handle.close()  # the weak reference is dead; nothing to do


if __name__ == '__main__':
  absltest.main()
