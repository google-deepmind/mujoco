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

import os
import tempfile

from absl.testing import absltest
from etils import epath
import mujoco

try:
  from mujoco.usd import exporter as exporter_module  # pylint: disable=g-import-not-at-top
except ModuleNotFoundError:
  exporter_module = None


class ExporterTest(absltest.TestCase):

  @absltest.skipIf(exporter_module is None, "USD library is not available.")
  def test_usd_export(self):

    output_dir_root = os.getenv(
        "TEST_UNDECLARED_OUTPUTS_DIR", tempfile.gettempdir()
    )
    output_dir_name = "usd_test"
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
        output_directory=output_dir_name,
        output_directory_root=output_dir_root,
        camera_names=["closeup"],
    )
    mujoco.mj_step(model, data)
    exporter.update_scene(data)
    exporter.save_scene("usda")

    with open(
        os.path.join(
            output_dir_root, f"{output_dir_name}/frames", "frame_1.usda"
        ),
        "r",
        encoding="utf-8",
    ) as f:
      golden_path = os.path.join(
          epath.resource_path("mujoco"), "testdata", "usd_golden.usda"
      )
      with open(golden_path, "r", encoding="utf-8") as golden_file:
        self.assertEqual(f.readlines(), golden_file.readlines())


if __name__ == "__main__":
  absltest.main()
