# Copyright 2026 DeepMind Technologies Limited
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
"""Tests for GIL-free operation under free-threaded Python."""

import sys
import sysconfig

from absl.testing import absltest
import mujoco  # pytype: disable=unused-import


class GilDisabledTest(absltest.TestCase):

  @absltest.skipUnless(
      sysconfig.get_config_var('Py_GIL_DISABLED'),
      'Not running under free-threaded Python',
  )
  def test_gil_is_disabled(self):
    self.assertFalse(sys._is_gil_enabled())  # pylint: disable=protected-access


if __name__ == '__main__':
  absltest.main()
