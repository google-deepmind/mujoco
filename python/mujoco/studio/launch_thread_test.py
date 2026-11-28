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
"""Tests for forwarding viewer-thread failures."""

import threading
import traceback
import weakref

from absl.testing import absltest
from mujoco.studio import launch_thread


class LaunchThreadTest(absltest.TestCase):

  def test_exception_releases_frame_locals_on_viewer_thread(self):
    created_on = []
    released_on = []

    class Resource:
      pass

    def fail_inner():
      resource = Resource()
      weakref.finalize(
          resource, lambda: released_on.append(threading.get_ident())
      )
      raise RuntimeError('inner')

    def target(unused_endpoint):
      created_on.append(threading.get_ident())
      resource = Resource()
      weakref.finalize(
          resource, lambda: released_on.append(threading.get_ident())
      )
      try:
        fail_inner()
      except RuntimeError as exc:
        raise ValueError('viewer failed') from exc

    handle = launch_thread.launch_thread(target)
    try:
      handle.close()
    except ValueError as exc:
      self.assertEqual(str(exc), 'viewer failed')
      self.assertIsInstance(exc.__cause__, RuntimeError)
      self.assertIn(
          'target', [f.name for f in traceback.extract_tb(exc.__traceback__)]
      )
      self.assertIn(
          'fail_inner',
          [f.name for f in traceback.extract_tb(exc.__cause__.__traceback__)],
      )
      self.assertLen(created_on, 1)
      self.assertEqual(released_on, created_on * 2)
    else:
      self.fail('Viewer error was not forwarded.')
    handle.close()


if __name__ == '__main__':
  absltest.main()
