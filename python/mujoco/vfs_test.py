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
import textwrap

from absl.testing import absltest
import mujoco


SIMPLE_XML = b"<mujoco/>"

XML_WITH_MESH = textwrap.dedent("""\
    <mujoco>
      <asset>
        <mesh name="box" file="box.obj"/>
      </asset>
      <worldbody>
        <body>
          <geom type="mesh" mesh="box"/>
        </body>
      </worldbody>
    </mujoco>
""").encode()

BOX_OBJ = textwrap.dedent("""\
    v -1 -1 -1
    v  1 -1 -1
    v  1  1 -1
    v -1  1 -1
    v -1 -1  1
    v  1 -1  1
    v  1  1  1
    v -1  1  1
    f 1 2 3 4
    f 5 8 7 6
    f 1 5 6 2
    f 2 6 7 3
    f 3 7 8 4
    f 4 8 5 1
""").encode()


class VfsLifecycleTest(absltest.TestCase):

  def test_create_and_close(self):
    vfs = mujoco.MjVfs()
    vfs.close()

  def test_context_manager(self):
    with mujoco.MjVfs() as vfs:
      self.assertIsNotNone(vfs)

  def test_double_close_is_safe(self):
    vfs = mujoco.MjVfs()
    vfs.close()
    vfs.close()

  def test_operations_after_close_raise(self):
    vfs = mujoco.MjVfs()
    vfs.close()
    with self.assertRaises(RuntimeError):
      vfs["model.xml"] = SIMPLE_XML


class VfsBufferTest(absltest.TestCase):

  def test_add_buffer(self):
    with mujoco.MjVfs() as vfs:
      vfs["model.xml"] = SIMPLE_XML
      self.assertIn("model.xml", vfs)

  def test_add_duplicate_raises(self):
    with mujoco.MjVfs() as vfs:
      vfs["model.xml"] = SIMPLE_XML
      with self.assertRaises(ValueError):
        vfs["model.xml"] = SIMPLE_XML

  def test_delete_buffer(self):
    with mujoco.MjVfs() as vfs:
      vfs["model.xml"] = SIMPLE_XML
      self.assertIn("model.xml", vfs)
      del vfs["model.xml"]
      self.assertNotIn("model.xml", vfs)

  def test_delete_missing_raises(self):
    with mujoco.MjVfs() as vfs:
      with self.assertRaises(KeyError):
        del vfs["nonexistent"]

  def test_contains_missing(self):
    with mujoco.MjVfs() as vfs:
      self.assertNotIn("nonexistent", vfs)


class VfsCompileTest(absltest.TestCase):

  def test_compile_simple_model(self):
    with mujoco.MjVfs() as vfs:
      vfs["model.xml"] = SIMPLE_XML
      model = mujoco.MjModel.from_xml_path("model.xml", vfs=vfs)
      self.assertEqual(model.nq, 0)

  def test_compile_model_with_mesh(self):
    with mujoco.MjVfs() as vfs:
      vfs["model.xml"] = XML_WITH_MESH
      vfs["box.obj"] = BOX_OBJ
      model = mujoco.MjModel.from_xml_path("model.xml", vfs=vfs)
      self.assertEqual(model.nmesh, 1)

  def test_spec_from_string_with_vfs(self):
    with mujoco.MjVfs() as vfs:
      vfs["box.obj"] = BOX_OBJ
      spec = mujoco.MjSpec.from_string(XML_WITH_MESH.decode(), vfs=vfs)
      model = spec.compile(vfs=vfs)
      self.assertEqual(model.nmesh, 1)

  def test_from_xml_string_with_vfs(self):
    with mujoco.MjVfs() as vfs:
      vfs["box.obj"] = BOX_OBJ
      model = mujoco.MjModel.from_xml_string(
          XML_WITH_MESH.decode(), vfs=vfs
      )
      self.assertEqual(model.nmesh, 1)

  def test_spec_compile_with_vfs(self):
    with mujoco.MjVfs() as vfs:
      vfs["box.obj"] = BOX_OBJ
      spec = mujoco.MjSpec.from_string(XML_WITH_MESH.decode())
      model = spec.compile(vfs=vfs)
      self.assertEqual(model.nmesh, 1)

  def test_spec_recompile_with_vfs(self):
    with mujoco.MjVfs() as vfs:
      vfs["box.obj"] = BOX_OBJ
      spec = mujoco.MjSpec.from_string(XML_WITH_MESH.decode())
      model1 = spec.compile(vfs=vfs)
      self.assertEqual(model1.ngeom, 1)

      body = spec.worldbody.add_body()
      body.add_geom(size=[1, 0, 0])
      model2 = spec.compile(vfs=vfs)
      self.assertEqual(model2.ngeom, 2)

  def test_long_lived_vfs_without_context(self):
    vfs = mujoco.MjVfs()
    vfs["box.obj"] = BOX_OBJ

    spec = mujoco.MjSpec.from_string(XML_WITH_MESH.decode())
    model1 = spec.compile(vfs=vfs)
    self.assertEqual(model1.nmesh, 1)

    spec.worldbody.add_body().add_geom(size=[1, 0, 0])
    model2 = spec.compile(vfs=vfs)
    self.assertEqual(model2.nmesh, 1)
    self.assertEqual(model2.ngeom, 2)

    vfs.close()

  def test_vfs_and_assets_raises(self):
    with mujoco.MjVfs() as vfs:
      vfs["model.xml"] = SIMPLE_XML
      with self.assertRaises(ValueError):
        mujoco.MjModel.from_xml_string(
            SIMPLE_XML.decode(), assets={"a": b"b"}, vfs=vfs
        )

  def test_backward_compat_assets_dict(self):
    model = mujoco.MjModel.from_xml_string(
        XML_WITH_MESH.decode(), assets={"box.obj": BOX_OBJ}
    )
    self.assertEqual(model.nmesh, 1)


class VfsAttachTest(absltest.TestCase):

  def test_attach_shared_vfs(self):
    child_xml = textwrap.dedent("""\
        <mujoco>
          <asset>
            <mesh name="child_mesh" file="box.obj"/>
          </asset>
          <worldbody>
            <body name="child">
              <geom type="mesh" mesh="child_mesh" size="1 1 1"/>
              <site name="attachment_site"/>
            </body>
          </worldbody>
        </mujoco>
    """)
    parent_xml = textwrap.dedent("""\
        <mujoco>
          <worldbody>
            <body name="parent">
              <site name="mount"/>
            </body>
          </worldbody>
        </mujoco>
    """)

    with mujoco.MjVfs() as vfs:
      vfs["box.obj"] = BOX_OBJ

      parent = mujoco.MjSpec.from_string(parent_xml)
      child = mujoco.MjSpec.from_string(child_xml)
      parent.attach(child, site="mount")
      model = parent.compile(vfs=vfs)
      self.assertEqual(model.nmesh, 1)
      self.assertEmpty(parent.assets)

  def test_attach_no_asset_dict_needed(self):
    child_xml = textwrap.dedent("""\
        <mujoco>
          <worldbody>
            <body name="child">
              <geom size="1"/>
              <site name="child_site"/>
            </body>
          </worldbody>
        </mujoco>
    """)
    parent_xml = textwrap.dedent("""\
        <mujoco>
          <worldbody>
            <body name="parent">
              <site name="mount"/>
            </body>
          </worldbody>
        </mujoco>
    """)

    with mujoco.MjVfs() as vfs:
      parent = mujoco.MjSpec.from_string(parent_xml)
      child = mujoco.MjSpec.from_string(child_xml)
      parent.attach(child, site="mount")
      model = parent.compile(vfs=vfs)
      self.assertGreater(model.nbody, 1)


if __name__ == "__main__":
  absltest.main()
