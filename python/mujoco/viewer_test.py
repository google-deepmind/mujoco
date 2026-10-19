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
"""Extremely minimal test of mujoco.viewer that just tries to import it."""

from unittest import mock

from absl.testing import absltest
from mujoco import viewer


class ViewerTest(absltest.TestCase):

  def test_launch_function_exists(self):
    self.assertIsNotNone(viewer.launch)


class TextUpdateTest(absltest.TestCase):

  def setUp(self):
    super().setUp()
    self.sim = mock.Mock()
    self.handle = viewer.Handle(self.sim, None, None, None, None)
    self.clock = self.enter_context(mock.patch.object(viewer.time, 'monotonic'))
    self.clock.return_value = 0.0

  def test_interval_skips_native_submission(self):
    self.handle.set_texts((None, None, 'first', None), update_interval=0.5)
    self.clock.return_value = 0.49
    self.handle.set_texts((None, None, 'skipped', None), update_interval=0.5)
    self.assertEqual(self.sim.set_texts.call_count, 1)
    self.clock.return_value = 0.5
    self.handle.set_texts((None, None, 'latest', None), update_interval=0.5)
    self.assertEqual(self.sim.set_texts.call_count, 2)
    self.assertEqual(self.sim.set_texts.call_args.args[0][0][2], 'latest')

  def test_default_submits_every_call_and_keeps_defaults(self):
    for _ in range(3):
      self.handle.set_texts((None, None, None, None))
    self.assertEqual(self.sim.set_texts.call_count, 3)
    self.sim.set_texts.assert_called_with([(
        viewer.mujoco.mjtFontScale.mjFONTSCALE_150,
        viewer.mujoco.mjtGridPos.mjGRID_TOPLEFT,
        '',
        '',
    )])

  def test_clear_resets_deadline(self):
    self.handle.set_texts((None, None, 'first', None), update_interval=10)
    self.handle.clear_texts()
    self.handle.set_texts((None, None, 'next', None), update_interval=10)
    self.assertEqual(self.sim.set_texts.call_count, 2)
    self.sim.clear_texts.assert_called_once()

  def test_unthrottled_final_update(self):
    self.handle.set_texts((None, None, 'first', None), update_interval=10)
    self.handle.set_texts((None, None, 'done', None))
    self.assertEqual(self.sim.set_texts.call_count, 2)

  def test_invalid_interval(self):
    for interval in [-1, float('nan'), float('inf')]:
      with self.assertRaises(ValueError):
        self.handle.set_texts([], update_interval=interval)
    self.sim.set_texts.assert_not_called()

  def test_destroyed_viewer(self):
    del self.sim
    self.handle.set_texts([], update_interval=1)
    self.handle.clear_texts()

  def test_failed_submission_does_not_advance_deadline(self):
    self.sim.set_texts.side_effect = RuntimeError('failed')
    with self.assertRaises(RuntimeError):
      self.handle.set_texts([], update_interval=10)
    self.sim.set_texts.side_effect = None
    self.handle.set_texts([], update_interval=10)
    self.assertEqual(self.sim.set_texts.call_count, 2)


if __name__ == '__main__':
  absltest.main()
