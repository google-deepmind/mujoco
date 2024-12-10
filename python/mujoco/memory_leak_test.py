# Copyright 2024 DeepMind Technologies Limited
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
"""Test that copying mujoco.MjData multiple times doesn't leak memory."""

import copy
import textwrap

from absl.testing import absltest
import mujoco


class MemoryLeakTest(absltest.TestCase):

  # Regression test for https://github.com/google-deepmind/mujoco/issues/1572
  def test_deepcopy_mjdata_leak(self):
    # MuJoCo model with textures that take up significant memory.
    model_xml = textwrap.dedent("""
    <mujoco>
      <asset>
        <material name="body"/>
        <texture name="grid" type="2d" builtin="checker"
            width="1024" height="1024" rgb1="0 0 0" rgb2="1 1 1"/>
        <material name="grid" texture="grid" texrepeat="1 1"
            texuniform="true" reflectance=".2"/>
      </asset>
      <worldbody>
          <geom size="0 0 .05" type="plane" material="grid"/>
      </worldbody>
    </mujoco>
    """)
    limit = self._memory_limit(4 * 2**32)
    try:
      model = mujoco.MjModel.from_xml_string(model_xml)
      data = mujoco.MjData(model)
      total = 0
      for _ in range(1000):
        mujoco.mj_step(model, data)
        last_data = copy.deepcopy(data)
        total += last_data.time
    finally:
      self._memory_limit(limit)

  def _memory_limit(self, limit_in_bytes: int) -> int:
    """Limits max memory usage, and returns previous limit."""
    soft = -1
    try:
      import resource  # pylint: disable=g-import-not-at-top

      soft, hard = resource.getrlimit(resource.RLIMIT_AS)
      resource.setrlimit(resource.RLIMIT_AS, (limit_in_bytes, hard))
    except (ImportError, ValueError):
      # On Windows or systems where setting resource limits fails, do nothing.
      pass
    return soft


if __name__ == "__main__":
  absltest.main()
