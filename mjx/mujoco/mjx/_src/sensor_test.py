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
"""Tests for sensor functions."""

from absl.testing import absltest
from absl.testing import parameterized
import jax
from jax import numpy as jp
import mujoco
from mujoco import mjx
from mujoco.mjx._src import test_util
from mujoco.mjx._src.types import ConeType
import numpy as np

# tolerance for difference between MuJoCo and MJX smooth calculations - mostly
# due to float precision
_TOLERANCE = 5e-5


def _assert_eq(a, b, name):
  tol = _TOLERANCE * 10  # avoid test noise
  err_msg = f'mismatch: {name}'
  np.testing.assert_allclose(a, b, err_msg=err_msg, atol=tol, rtol=tol)


def _assert_attr_eq(a, b, attr):
  _assert_eq(getattr(a, attr), getattr(b, attr), attr)


class SensorTest(parameterized.TestCase):

  @parameterized.product(
      filename=['sensor/model.xml', 'sensor/sensor.xml'],
      cone_type=list(ConeType),
  )
  def test_sensor(self, filename, cone_type):
    """Tests MJX sensor functions match MuJoCo sensor functions."""
    m = test_util.load_test_file(filename)
    m.opt.cone = cone_type
    d = mujoco.MjData(m)
    # give the system a little kick to ensure we have non-identity rotations
    d.qvel = 0.1 * np.random.random(m.nv)
    # apply external forces
    d.xfrc_applied = 0.1 * np.random.random(d.xfrc_applied.shape)
    # apply control for activation dynamics
    d.ctrl = np.clip(
        0.1 * np.random.random(m.nu),
        m.actuator_ctrlrange[:, 0],
        m.actuator_ctrlrange[:, 1],
    )
    mujoco.mj_step(m, d, 100)
    mujoco.mj_forward(m, d)

    mx = mjx.put_model(m)
    dx = mjx.put_data(m, d).tree_replace({
        'sensordata': jp.zeros_like(d.sensordata),
        '_impl.subtree_linvel': jp.zeros_like(d.subtree_linvel),
        '_impl.subtree_angmom': jp.zeros_like(d.subtree_angmom),
        '_impl.cacc': jp.zeros_like(d.cacc),
        '_impl.cfrc_int': jp.zeros_like(d.cfrc_int),
        '_impl.cfrc_ext': jp.zeros_like(d.cfrc_ext),
    })
    dx = jax.jit(mjx.sensor_pos)(mx, dx)
    dx = jax.jit(mjx.sensor_vel)(mx, dx)
    dx = jax.jit(mjx.sensor_acc)(mx, dx)

    _assert_eq(d.sensordata, dx.sensordata, 'sensordata')

  def test_disable_sensor(self):
    """Tests disabling sensor."""
    m = test_util.load_test_file('sensor/sensor.xml')
    # disable sensors
    m.opt.disableflags = m.opt.disableflags | mjx.DisableBit.SENSOR
    d = mujoco.MjData(m)
    # give the system a little kick to ensure we have non-identity rotations
    d.qvel = np.random.random(m.nv)
    mujoco.mj_step(m, d, 10)  # let dynamics get state significantly non-zero
    mx = mjx.put_model(m)
    dx = mjx.put_data(m, d)
    # random sensor values
    random_sensor = jp.array(np.random.random(dx.sensordata.shape))
    dx = dx.replace(sensordata=random_sensor)
    # call sensor functions
    dx = jax.jit(mjx.forward)(mx, dx)
    # sensor values
    _assert_eq(random_sensor, dx.sensordata, 'sensordata')

  def test_unsupported_sensor(self):
    """Tests unsupported sensor raises error."""
    m = mujoco.MjModel.from_xml_string("""
      <mujoco>
        <worldbody>
          <body>
            <joint type="hinge"/>
            <geom name="geom0" size="0.1"/>
            <body>
              <joint type="hinge"/>
              <geom name="geom1" size="0.25"/>
            </body>
          </body>
        </worldbody>
        <sensor>
          <distance name="distance" geom1="geom0" geom2="geom1"/>
        </sensor>
      </mujoco>
    """)
    with self.assertRaises(NotImplementedError):
      mjx.put_model(m)


if __name__ == '__main__':
  absltest.main()
