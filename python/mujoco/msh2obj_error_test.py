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
"""Regression tests for malformed legacy MSH diagnostics."""

import pathlib
import tempfile

from absl.testing import absltest
from mujoco import msh2obj
import numpy as np


class Msh2ObjErrorTest(absltest.TestCase):

  def test_texcoord_error_reports_expected_count(self):
    with tempfile.TemporaryDirectory() as tmpdir:
      path = pathlib.Path(tmpdir) / 'bad.msh'
      # Header: no vertices/normals/faces, five texcoords expected. Only one
      # (two floats) is serialized so the texcoord-size validation is reached.
      header = np.array([0, 0, 5, 0], dtype=np.int32)
      texcoord = np.array([0.25, 0.75], dtype=np.float32)
      path.write_bytes(header.tobytes() + texcoord.tobytes())

      with self.assertRaisesRegex(
          ValueError, r'Invalid number of texcoords: 2 != 2\*5\.'
      ):
        msh2obj.Msh.create(path)


if __name__ == '__main__':
  absltest.main()
