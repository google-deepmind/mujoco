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

import glob
import ntpath
import os
import pathlib
import tempfile
from unittest import mock

from absl.testing import absltest
from etils import epath

import mujoco

try:
  from pxr import Usd  # pylint: disable=g-import-not-at-top
  from pxr import UsdShade  # pylint: disable=g-import-not-at-top

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

  @absltest.skipIf(exporter_module is None, "USD library is not available.")
  def test_invalid_usd_identifier_names(self):
    output_dir_root = self.create_tempdir().full_path
    xml = """
<mujoco>
  <asset>
    <mesh name="mesh/tri-0" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/>
  </asset>
  <worldbody>
    <camera name="robot/track-cam.0" pos="0 -6 0" xyaxes="1 0 0 0 1 100"/>
    <site name="site/a" pos="0 0 0"/>
    <site name="site/b" pos="0 0 1"/>
    <geom name="/robot//arm/" type="box" size="1 1 1"/>
    <geom name="robot/box" type="box" size="1 1 1"/>
    <geom name="robot//box" type="box" size="1 1 1"/>
    <geom name="robot_box" type="box" size="1 1 1"/>
    <geom name="link-1.0" type="sphere" size="0.5"/>
    <geom name="0_leading_digit" type="mesh" mesh="mesh/tri-0"/>
  </worldbody>
  <tendon>
    <spatial name="cable/main-0" width="0.02">
      <site site="site/a"/>
      <site site="site/b"/>
    </spatial>
  </tendon>
</mujoco>
"""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    exporter = exporter_module.USDExporter(
        model,
        output_directory="usd_sanitized_test",
        output_directory_root=output_dir_root,
        camera_names=["robot/track-cam.0"],
    )
    mujoco.mj_step(model, data)
    exporter.update_scene(data)
    exporter.add_camera(
        pos=[0.0, -3.0, 1.0],
        rotation_xyz=[0.0, 0.0, 0.0],
        obj_name="extra/cam-1",
    )
    exporter.add_light(
        pos=[0.0, 0.0, 2.0],
        intensity=500,
        obj_name="key/light-0",
        light_type="sphere",
    )
    exporter.add_light(
        pos=[0.0, 0.0, 0.0],
        intensity=250,
        obj_name="sky.dome/0",
        light_type="dome",
    )
    exporter.save_scene("usda")

    prim_types = {
        str(prim.GetPath()): prim.GetTypeName()
        for prim in exporter.stage.Traverse()
    }
    self.assertContainsSubset(
        {
            (
                "/World/Mesh_Xform__robot__arm__id0_geom"
                "/Mesh__robot__arm__id0_geom"
            ): "Mesh",
            (
                "/World/Mesh_Xform_robot_box_id1_geom/Mesh_robot_box_id1_geom"
            ): "Mesh",
            (
                "/World/Mesh_Xform_robot__box_id2_geom"
                "/Mesh_robot__box_id2_geom"
            ): "Mesh",
            (
                "/World/Mesh_Xform_robot_box_id3_geom/Mesh_robot_box_id3_geom"
            ): "Mesh",
            (
                "/World/Mesh_Xform_link_1_0_id4_geom/Mesh_link_1_0_id4_geom"
            ): "Mesh",
            (
                "/World/Mesh_Xform___leading_digit_id5_geom"
                "/Mesh___leading_digit_id5_geom"
            ): "Mesh",
            "/World/Mesh_Xform_cable_main_0_id0_tendon_segid8": "Xform",
            (
                "/World/Camera_Xform_robot_track_cam_0"
                "/Camera_robot_track_cam_0"
            ): "Camera",
            "/World/Camera_Xform_extra_cam_1/Camera_extra_cam_1": "Camera",
            "/World/Light_Xform_key_light_0/Light_key_light_0": "SphereLight",
            "/World/Light_Xform_sky_dome_0/Light_sky_dome_0": "DomeLight",
        },
        prim_types,
    )

  def _export_textured_scene_texture_paths(self):
    """Exports a textured scene and returns its texture asset paths."""
    output_dir_root = self.create_tempdir().full_path
    xml = """
<mujoco>
  <asset>
    <texture name="checker" type="2d" builtin="checker" rgb1="0.9 0.9 0.9"
             rgb2="0.1 0.1 0.1" width="64" height="64"/>
    <material name="floor_mat" texture="checker"/>
  </asset>
  <worldbody>
    <camera name="cam" pos="0 -4 1.5" xyaxes="1 0 0 0 0.35 1"/>
    <geom type="plane" size="5 5 0.1" material="floor_mat"/>
  </worldbody>
</mujoco>
"""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    exporter = exporter_module.USDExporter(
        model,
        output_directory="usd_texture_paths",
        output_directory_root=output_dir_root,
        camera_names=["cam"],
    )
    mujoco.mj_step(model, data)
    exporter.update_scene(data)
    exporter.save_scene("usda")
    frame_paths = glob.glob(
        os.path.join(output_dir_root, "**", "frame_*.usda"), recursive=True
    )
    self.assertLen(frame_paths, 1)
    stage = Usd.Stage.Open(frame_paths[0])
    paths = []
    for prim in stage.Traverse():
      if prim.IsA(UsdShade.Shader):
        file_input = UsdShade.Shader(prim).GetInput("file")
        if file_input:
          paths.append(file_input.Get().path)
    return paths

  @absltest.skipIf(exporter_module is None, "USD library is not available.")
  def test_texture_asset_paths_use_forward_slashes(self):
    paths = self._export_textured_scene_texture_paths()
    self.assertEqual(paths, ["../assets/texture_0.png"])

  @absltest.skipIf(exporter_module is None, "USD library is not available.")
  def test_texture_asset_paths_use_forward_slashes_with_windows_paths(self):
    # Simulates the Windows path flavor (backslash separators) on any host,
    # only inside the exporter module: its `os` is replaced by a proxy and its
    # `pathlib` by a Windows flavor. Global os.path, pxr, glob and tempfile
    # keep their native behavior.
    native_os = os

    class WindowsPathProxy:

      def __getattr__(self, name):
        return getattr(native_os.path, name)

      def relpath(self, *args, **kwargs):
        return "..\\assets"

      def join(self, first, *rest):
        if first.startswith(".."):
          return ntpath.join(first, *rest)
        return native_os.path.join(first, *rest)

    class OsProxy:
      path = WindowsPathProxy()

      def __getattr__(self, name):
        return getattr(native_os, name)

    with mock.patch.object(exporter_module, "os", OsProxy()):
      with mock.patch.object(
          exporter_module,
          "pathlib",
          mock.Mock(PurePath=pathlib.PureWindowsPath),
          create=True,
      ):
        paths = self._export_textured_scene_texture_paths()
    self.assertEqual(paths, ["../assets/texture_0.png"])


if __name__ == "__main__":
  absltest.main()
