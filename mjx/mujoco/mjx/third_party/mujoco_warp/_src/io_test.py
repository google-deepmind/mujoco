# Copyright 2025 The Newton Developers
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

"""Tests for io functions."""

import mujoco
import numpy as np
import warp as wp
from absl.testing import absltest
from absl.testing import parameterized

import mujoco_warp as mjwarp
from mujoco.mjx.third_party.mujoco_warp import test_data


def _assert_eq(a, b, name):
  tol = 5e-4
  err_msg = f"mismatch: {name}"
  np.testing.assert_allclose(a, b, err_msg=err_msg, atol=tol, rtol=tol)


# NOTE: modify io_jax_test _IO_TEST_MODELS if changed here.
_IO_TEST_MODELS = (
  "pendula.xml",
  "collision_sdf/tactile.xml",
  "flex/floppy.xml",
  "actuation/tendon_force_limit.xml",
  "hfield/hfield.xml",
)


class IOTest(parameterized.TestCase):
  def test_make_put_data(self):
    """Tests that make_data and put_data are producing the same shapes for all arrays."""
    mjm, _, _, d = test_data.fixture("pendula.xml")
    md = mjwarp.make_data(mjm, nconmax=512, njmax=512)

    # same number of fields
    self.assertEqual(len(d.__dict__), len(md.__dict__))

    # test shapes for all arrays
    for attr, val in md.__dict__.items():
      if isinstance(val, wp.array):
        self.assertEqual(val.shape, getattr(d, attr).shape, f"{attr} shape mismatch")

  def test_get_data_into_m(self):
    mjm = mujoco.MjModel.from_xml_string("""
      <mujoco>
        <worldbody>
          <body pos="0 0 0" >
            <geom type="box" pos="0 0 0" size=".5 .5 .5" />
            <joint type="hinge" />
          </body>
          <body pos="0 0 0.1">
            <geom type="sphere" size="0.5"/>
            <freejoint/>
          </body>
        </worldbody>
      </mujoco>
    """)

    mjd = mujoco.MjData(mjm)
    mujoco.mj_forward(mjm, mjd)

    mjd_ref = mujoco.MjData(mjm)
    mujoco.mj_forward(mjm, mjd_ref)

    m = mjwarp.put_model(mjm)
    d = mjwarp.put_data(mjm, mjd)

    mjd.qLD.fill(-123)
    mjd.qM.fill(-123)

    mjwarp.get_data_into(mjd, mjm, d)
    np.testing.assert_allclose(mjd.qLD, mjd_ref.qLD)
    np.testing.assert_allclose(mjd.qM, mjd_ref.qM)

  def test_ellipsoid_fluid_model(self):
    with self.assertRaises(NotImplementedError):
      mjm = mujoco.MjModel.from_xml_string(
        """
      <mujoco>
        <option density="1"/>
        <worldbody>
          <body>
            <geom type="sphere" size=".1" fluidshape="ellipsoid"/>
            <freejoint/>
          </body>
        </worldbody>
      </mujoco>
      """
      )
      mjwarp.put_model(mjm)

  def test_jacobian_auto(self):
    mjm = mujoco.MjModel.from_xml_string("""
      <mujoco>
        <option jacobian="auto"/>
        <worldbody>
          <replicate count="11">
          <body>
            <geom type="sphere" size=".1"/>
            <freejoint/>
            </body>
          </replicate>
        </worldbody>
      </mujoco>
    """)
    mjwarp.put_model(mjm)

  def test_put_data_qLD(self):
    mjm = mujoco.MjModel.from_xml_string("""
    <mujoco>
      <worldbody>
        <body>
          <geom type="sphere" size="1"/>
          <joint type="hinge"/>
        </body>
      </worldbody>
    </mujoco>
    """)
    mjd = mujoco.MjData(mjm)
    d = mjwarp.put_data(mjm, mjd)
    self.assertTrue((d.qLD.numpy() == 0.0).all())

    mujoco.mj_forward(mjm, mjd)
    mjd.qM[:] = 0.0
    d = mjwarp.put_data(mjm, mjd)
    self.assertTrue((d.qLD.numpy() == 0.0).all())

    mujoco.mj_forward(mjm, mjd)
    mjd.qLD[:] = 0.0
    d = mjwarp.put_data(mjm, mjd)
    self.assertTrue((d.qLD.numpy() == 0.0).all())

  def test_noslip_solver(self):
    with self.assertRaises(NotImplementedError):
      test_data.fixture(
        xml="""
      <mujoco>
        <option noslip_iterations="1"/>
      </mujoco>
      """
      )

  @parameterized.parameters(*_IO_TEST_MODELS)
  def test_reset_data(self, xml):
    reset_datafield = [
      "ncon",
      "ne",
      "nf",
      "nl",
      "nefc",
      "time",
      "energy",
      "qpos",
      "qvel",
      "act",
      "ctrl",
      "eq_active",
      "qfrc_applied",
      "xfrc_applied",
      "qacc",
      "qacc_warmstart",
      "act_dot",
      "sensordata",
      "mocap_pos",
      "mocap_quat",
      "qM",
    ]

    mjm, mjd, m, d = test_data.fixture(xml)
    nconmax = d.nconmax

    # data fields
    for arr in reset_datafield:
      attr = getattr(d, arr)
      if attr.dtype == float:
        attr.fill_(wp.nan)
      else:
        attr.fill_(-1)

    for arr in d.contact.__dataclass_fields__:
      attr = getattr(d.contact, arr)
      if attr.dtype == float:
        attr.fill_(wp.nan)
      else:
        attr.fill_(-1)

    mujoco.mj_resetData(mjm, mjd)

    # set ncon in order to zero all contact memory
    wp.copy(d.ncon, wp.array([nconmax], dtype=int))
    mjwarp.reset_data(m, d)

    for arr in reset_datafield:
      d_arr = getattr(d, arr).numpy()
      for i in range(d_arr.shape[0]):
        di_arr = d_arr[i]
        if arr == "qM":
          di_arr = di_arr.reshape(-1)[: mjd.qM.size]
        _assert_eq(di_arr, getattr(mjd, arr), arr)

    for arr in d.contact.__dataclass_fields__:
      _assert_eq(getattr(d.contact, arr).numpy(), 0.0, arr)

  def test_sdf(self):
    """Tests that an SDF can be loaded."""
    mjm, mjd, m, d = test_data.fixture("collision_sdf/cow.xml")

    self.assertIsInstance(m.oct_aabb, wp.array)
    self.assertEqual(m.oct_aabb.dtype, wp.vec3)
    self.assertEqual(len(m.oct_aabb.shape), 2)
    if m.oct_aabb.size > 0:
      self.assertEqual(m.oct_aabb.shape[1], 2)

  @parameterized.parameters(
    ('<distance geom1="plane" geom2="sphere"/>', NotImplementedError),
    ('<distance geom1="sphere" geom2="plane"/>', NotImplementedError),
    ('<distance geom1="hfield" geom2="sphere"/>', ValueError),
    ('<distance geom1="sphere" geom2="hfield"/>', ValueError),
  )
  def test_collision_sensors(self, sensor, err):
    """Tests for collision sensors that are not implemented."""
    with self.assertRaises(err):
      test_data.fixture(
        xml=f"""
      <mujoco>
        <asset>
          <hfield name="hfield" nrow="2" ncol="2" size="1 1 1 1"/>
        </asset>
        <worldbody>
          <geom name="plane" type="plane" size="10 10 .01"/>
          <geom name="hfield" type="hfield" hfield="hfield"/>
          <body>
            <geom name="sphere" type="sphere" size=".1"/>
            <joint type="slide" axis="0 0 1"/>
          </body>
        </worldbody>
        <sensor>
          {sensor}
        </sensor>
      </mujoco>
      """
      )

  def test_implicit_integrator_fluid_model(self):
    """Tests for implicit integrator with fluid model."""
    with self.assertRaises(NotImplementedError):
      test_data.fixture(
        xml="""
        <mujoco>
          <option viscosity="1" density="1" integrator="implicitfast"/>
          <worldbody>
            <body>
              <geom type="sphere" size=".1"/>
              <freejoint/>
            </body>
          </worldbody>
        </mujoco>
        """
      )

  def test_plugin(self):
    with self.assertRaises(NotImplementedError):
      test_data.fixture(
        xml="""
      <mujoco>
        <extension>
          <plugin plugin="mujoco.pid"/>
          <plugin plugin="mujoco.sensor.touch_grid"/>
          <plugin plugin="mujoco.elasticity.cable"/>
        </extension>
        <worldbody>
          <geom type="plane" size="10 10 .001"/>
          <body>
            <joint name="joint" type="slide"/>
            <geom type="sphere" size=".1"/>
            <site name="site"/>
          </body>
          <composite type="cable" curve="s" count="41 1 1" size="1" offset="-.3 0 .6" initial="none">
            <plugin plugin="mujoco.elasticity.cable">
              <config key="twist" value="1e7"/>
              <config key="bend" value="4e6"/>
              <config key="vmax" value="0.05"/>
            </plugin>
            <joint kind="main" damping=".015"/>
            <geom type="capsule" size=".005" rgba=".8 .2 .1 .1" condim="1"/>
          </composite>
        </worldbody>
        <actuator>
          <plugin plugin="mujoco.pid" joint="joint"/>
        </actuator>
        <sensor>
          <plugin plugin="mujoco.sensor.touch_grid" objtype="site" objname="site">
            <config key="size" value="7 7"/>
            <config key="fov" value="45 45"/>
            <config key="gamma" value="0"/>
            <config key="nchannel" value="3"/>
          </plugin>
        </sensor>
      </mujoco>
      """
      )


if __name__ == "__main__":
  wp.init()
  absltest.main()
