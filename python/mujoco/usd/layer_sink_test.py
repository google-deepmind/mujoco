# Copyright 2025 DeepMind Technologies Limited
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

from absl.testing import absltest
from absl.testing import parameterized
import mujoco
import numpy as np
from mujoco.usd import layer_sink
from mujoco.usd import test_utils
from mujoco.usd import writer

from pxr import Sdf
from pxr import Usd
from pxr import UsdPhysics


_LAYER_SINK_XFORM_ATTRS = (
    "xformOp:translate:mujoco",
    "xformOp:orient:mujoco",
)


class MujocoUsdLayerSinkTest(parameterized.TestCase):

  @parameterized.named_parameters(
      dict(
          testcase_name="simple_cube",
          test_data_fn=test_utils.simple_falling_cube,
      ),
      dict(
          testcase_name="nested_hierarchy",
          test_data_fn=test_utils.simple_nested_hierarchy,
      ),
  )
  def test_layer_sink(self, test_data_fn):
    test_data = test_data_fn()

    physics_layer = Sdf.Layer.CreateAnonymous()
    physics_layer.startTimeCode = 0
    physics_layer.timeCodesPerSecond = 1.0 / test_data.model.opt.timestep
    test_data.stage.GetRootLayer().subLayerPaths.append(
        physics_layer.identifier
    )
    test_data.stage.SetEditTarget(physics_layer)

    usd_writer = writer.UsdWriter(
        test_data.stage, test_data.spec, test_data.model
    )
    sink = layer_sink.LayerSink(test_data.stage)
    usd_writer.add_sink(sink.update)

    n_steps = 100
    for _ in range(n_steps):
      mujoco.mj_step(test_data.model, test_data.data)
      usd_writer.update(test_data.data)

    for prim in test_data.stage.Traverse():
      if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        for attr_name in _LAYER_SINK_XFORM_ATTRS:
          attr_path = prim.GetPath().AppendProperty(attr_name)
          attr_spec = physics_layer.GetPropertyAtPath(attr_path)
          self.assertIsNotNone(attr_spec)
          n_samples = physics_layer.GetNumTimeSamplesForPath(attr_path)
          self.assertEqual(n_samples, n_steps)


  def test_records_unnamed_sibling_bodies(self):
    spec = mujoco.MjSpec.from_string("""
      <mujoco model="unnamed_test">
        <worldbody>
          <body name="named" pos="0 0 2">
            <freejoint/>
            <geom type="sphere" size="0.1"/>
          </body>
          <body pos="1 0 2">
            <freejoint/>
            <geom type="sphere" size="0.1"/>
          </body>
          <body pos="2 0 2">
            <freejoint/>
            <geom type="sphere" size="0.1"/>
          </body>
        </worldbody>
      </mujoco>
    """)
    model = spec.compile()
    data = mujoco.MjData(model)
    stage = Usd.Stage.CreateInMemory()
    stage.SetTimeCodesPerSecond(1.0 / model.opt.timestep)
    usd_writer = writer.UsdWriter(stage, spec, model)
    sink = layer_sink.LayerSink(stage)
    paths = []

    def capture_paths(unused_data, body_paths):
      paths[:] = body_paths

    usd_writer.add_sink(capture_paths)
    usd_writer.add_sink(sink.update)
    for _ in range(3):
      mujoco.mj_step(model, data)
      usd_writer.update(data)

    self.assertLen(paths, model.nbody)
    self.assertTrue(paths[0].isEmpty)
    self.assertLen(set(paths[1:]), 3)
    self.assertEqual(paths[1], Sdf.Path("/unnamed_test/named"))
    for body_id, path in enumerate(paths[1:], start=1):
      self.assertFalse(path.isEmpty)
      prim = stage.GetPrimAtPath(path)
      self.assertTrue(prim)
      for attr_name in _LAYER_SINK_XFORM_ATTRS:
        attr = prim.GetAttribute(attr_name)
        self.assertLen(attr.GetTimeSamples(), 3)
      translation = prim.GetAttribute(_LAYER_SINK_XFORM_ATTRS[0])
      sample = translation.Get(translation.GetTimeSamples()[-1])
      np.testing.assert_allclose(sample, data.xpos[body_id], atol=1e-6)


if __name__ == "__main__":
  absltest.main()
