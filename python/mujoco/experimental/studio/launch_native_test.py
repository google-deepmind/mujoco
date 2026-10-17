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
"""Tests for the native Studio viewer launchers.

These tests exercise the launcher control flow (threading, shutdown and error
propagation) without creating a real desktop window: ``run_native_viewer`` is
replaced with a fake that records the thread it runs on and returns once the
simulation side sends an ``ExitEvent``. The real rendering path (Filament, the
``NSWindow`` on macOS) is not exercised here.
"""

import threading
import time
from unittest import mock

from absl.testing import absltest
from mujoco.experimental.studio import launch_native
from mujoco.experimental.studio import launch_passive
from mujoco.experimental.studio import messages
from mujoco.experimental.studio import viewer_handle
from mujoco.experimental.studio import viewer_protocol


class _FakeViewer:
  """Stand-in for ``run_native_viewer``.

  Records the thread it is called on and the viewer plugins it received, then
  blocks (like the real viewer loop) until the simulation side sends an
  ``ExitEvent``, so the launcher's shutdown path runs exactly as in production.
  """

  def __init__(self) -> None:
    self.thread: threading.Thread | None = None
    self.plugins = None
    self.started = threading.Event()

  def __call__(self, config, endpoint, plugins=None) -> None:
    del config  # Unused by the fake.
    self.thread = threading.current_thread()
    self.plugins = plugins
    self.started.set()
    deadline = time.time() + 10.0
    while time.time() < deadline:
      for event in endpoint.get_sim_events():
        if isinstance(event, messages.ExitEvent):
          return
      time.sleep(0.002)
    raise AssertionError('fake viewer timed out waiting for an ExitEvent')


class LaunchNativeTest(absltest.TestCase):

  def test_public_functions_exist(self):
    self.assertTrue(callable(launch_native.launch))
    self.assertTrue(callable(launch_native.run))
    self.assertTrue(callable(launch_passive.launch))
    self.assertTrue(callable(launch_passive.run))

  def test_launch_raises_on_macos(self):
    # The non-blocking launcher cannot create the window on a worker thread on
    # macOS, so it must fail loudly rather than silently (issue #3579).
    with mock.patch('sys.platform', 'darwin'):
      with self.assertRaises(RuntimeError) as ctx:
        launch_native.launch(viewer_protocol.ViewerConfig())
    message = str(ctx.exception)
    self.assertIn('main thread', message)
    self.assertIn('run()', message)

  def test_launch_runs_viewer_on_worker_thread_off_macos(self):
    fake = _FakeViewer()
    with mock.patch('sys.platform', 'linux'), mock.patch.object(
        launch_native, 'run_native_viewer', fake
    ):
      handle = launch_native.launch(viewer_protocol.ViewerConfig())
      self.assertIsInstance(handle, viewer_handle.ViewerHandle)
      self.assertTrue(fake.started.wait(timeout=5.0))
      handle.close()  # Joins the viewer thread via the launcher's shutdown_fn.
    self.assertIsNotNone(fake.thread)
    self.assertIsNot(fake.thread, threading.main_thread())

  def test_run_on_main_thread_runs_viewer_on_calling_thread(self):
    fake = _FakeViewer()
    record = {}

    def sim_loop(handle):
      del handle  # Unused.
      record['sim_thread'] = threading.current_thread()

    with mock.patch.object(launch_native, 'run_native_viewer', fake):
      launch_native._run_viewer_on_main_thread(
          viewer_protocol.ViewerConfig(), sim_loop
      )

    # The viewer ran on the calling (here, the main) thread; the sim did not.
    self.assertIs(fake.thread, threading.main_thread())
    self.assertIn('sim_thread', record)
    self.assertIsNot(record['sim_thread'], threading.main_thread())

  def test_run_on_main_thread_joins_sim_worker(self):
    fake = _FakeViewer()
    record = {}

    def sim_loop(handle):
      del handle  # Unused.
      record['sim_thread'] = threading.current_thread()

    with mock.patch.object(launch_native, 'run_native_viewer', fake):
      launch_native._run_viewer_on_main_thread(
          viewer_protocol.ViewerConfig(), sim_loop
      )

    # The worker is joined before the launcher returns.
    self.assertFalse(record['sim_thread'].is_alive())

  def test_run_on_main_thread_forwards_viewer_plugins(self):
    fake = _FakeViewer()
    sentinel = object()

    def sim_loop(handle):
      del handle  # Unused.

    with mock.patch.object(launch_native, 'run_native_viewer', fake):
      launch_native._run_viewer_on_main_thread(
          viewer_protocol.ViewerConfig(), sim_loop, viewer_plugins=[sentinel]
      )

    self.assertEqual(fake.plugins, [sentinel])

  def test_run_on_main_thread_propagates_sim_error(self):
    fake = _FakeViewer()

    def sim_loop(handle):
      del handle  # Unused.
      raise ValueError('sim boom')

    with mock.patch.object(launch_native, 'run_native_viewer', fake):
      with self.assertRaisesRegex(ValueError, 'sim boom'):
        launch_native._run_viewer_on_main_thread(
            viewer_protocol.ViewerConfig(), sim_loop
        )

    # The viewer loop was still asked to stop despite the sim-side failure.
    self.assertIsNotNone(fake.thread)

  def test_run_routes_to_main_thread_launcher_on_macos(self):
    called = {}

    def fake_main(config, sim_loop, *, viewer_plugins=None, sim_plugins=None):
      del viewer_plugins, sim_plugins  # Unused.
      called['config'] = config
      called['sim_loop'] = sim_loop

    with mock.patch('sys.platform', 'darwin'), mock.patch.object(
        launch_native, '_run_viewer_on_main_thread', fake_main
    ):
      config = viewer_protocol.ViewerConfig()
      launch_native.run(config)

    self.assertIs(called.get('config'), config)
    self.assertTrue(callable(called.get('sim_loop')))

  def test_run_routes_through_launch_off_macos(self):
    events = []

    class _FakeHandle:

      def __enter__(self):
        return self

      def __exit__(self, *args):
        events.append('exit')
        return False

    fake_handle = _FakeHandle()
    with mock.patch('sys.platform', 'linux'), mock.patch.object(
        launch_native, 'launch', return_value=fake_handle
    ) as launch_mock, mock.patch.object(
        viewer_handle, 'run_sim_loop'
    ) as run_sim_loop:
      config = viewer_protocol.ViewerConfig()
      launch_native.run(config)

    launch_mock.assert_called_once()
    run_sim_loop.assert_called_once_with(fake_handle)
    self.assertIn('exit', events)

  def test_run_rejects_model_and_model_path(self):
    with self.assertRaises(ValueError):
      launch_native.run(
          viewer_protocol.ViewerConfig(), model=object(), model_path='m.xml'
      )


class LaunchPassiveTest(absltest.TestCase):

  def test_run_delegates_to_native_for_desktop(self):
    with mock.patch.object(launch_native, 'run') as native_run:
      config = viewer_protocol.ViewerConfig()
      launch_passive.run(config, model_path='model.xml')
    native_run.assert_called_once()
    args, kwargs = native_run.call_args
    self.assertIs(args[0], config)
    self.assertEqual(kwargs['model_path'], 'model.xml')

  def test_run_delegates_to_web_for_web_gfx(self):
    from mujoco.experimental.studio import launch_web  # pylint: disable=g-import-not-at-top

    with mock.patch.object(launch_web, 'run') as web_run:
      config = viewer_protocol.ViewerConfig(gfx='web')
      launch_passive.run(config)
    web_run.assert_called_once()
    args, _ = web_run.call_args
    self.assertIs(args[0], config)


if __name__ == '__main__':
  absltest.main()
