# Copyright 2023 DeepMind Technologies Limited
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
"""Tests for the test_util."""

from absl.testing import absltest
from etils import epath
from mujoco.mjx._src import test_util


class TestUtilTest(absltest.TestCase):

  def test_files_in_test_data_match(self):
    directory = epath.resource_path('mujoco.mjx') / 'test_data'
    files = set([f.name for f in directory.glob('*.xml')])
    self.assertSetEqual(
        files,
        set(test_util.TEST_FILES),
        msg=(
            '`_test_util.TEST_FILES` must match the files in the '
            'test_data/*.xml directory'
        ),
    )


if __name__ == '__main__':
  absltest.main()
