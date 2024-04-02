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
"""Tests for the MuJoCo USD Exporter."""

import logging
import os

import mujoco

from absl.testing import absltest
from etils import epath

# Open3D and USD are not fully supported on all MuJoCo architectures.
# pylint: disable=python.style(g-import-not-at-top)
execute_test = True
try:
  from mujoco.usd import exporter as exporter_module
  from pxr import Usd
except ImportError:
  logging.warning('Skipping test due to missing import')
  execute_test = False
# pylint: enable=python.style(g-import-not-at-top)

class ExporterTest(absltest.TestCase):

  def test_usd_export(self):
    if not execute_test:
      return

    output_dir = os.getenv('TEST_UNDECLARED_OUTPUTS_DIR')
    xml = """
<mujoco>
  <worldbody>
    <camera name="closeup" pos="0 -6 0" xyaxes="1 0 0 0 1 100"/>
    <geom name="white_box" type="box" size="1 1 1" rgba="1 1 1 1"/>
  </worldbody>
</mujoco>
"""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    exporter = exporter_module.USDExporter(
        model,
        output_directory_name="mujoco_usdpkg",
        output_directory_root=output_dir,
    )
    exporter.update_scene(data)
    exporter.save_scene("export.usda")

    with open(os.path.join(
        output_dir, "mujoco_usdpkg/frames", "frame_1_.export.usda"), "r") as f:
      golden_path = os.path.join(
          epath.resource_path("mujoco"), "testdata", "usd_golden.usda")
      with open(golden_path, "r") as golden_file:
        self.assertEqual(f.read(), golden_file.read())


if __name__ == "__main__":
  absltest.main()
