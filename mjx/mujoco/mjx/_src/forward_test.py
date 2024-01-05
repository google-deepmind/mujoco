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
"""Tests for forward functions."""

from absl.testing import absltest
import jax
import mujoco
from mujoco import mjx
from mujoco.mjx._src import test_util
import numpy as np


# tolerance for difference between MuJoCo and MJX forward calculations - mostly
# due to float precision
_TOLERANCE = 1e-5


def _assert_eq(a, b, name):
  tol = _TOLERANCE * 10  # avoid test noise
  err_msg = f'mismatch: {name}'
  np.testing.assert_allclose(a, b, err_msg=err_msg, atol=tol, rtol=tol)


def _assert_attr_eq(a, b, attr):
  _assert_eq(getattr(a, attr), getattr(b, attr), attr)


class ForwardTest(absltest.TestCase):

  def test_forward(self):
    m = test_util.load_test_file('constraints.xml')
    d = mujoco.MjData(m)
    # apply some control and xfrc input
    d.ctrl = np.array([-18, 0.59, 0.47])
    d.xfrc_applied[0, 2] = 0.1  # torque
    d.xfrc_applied[1, 4] = 0.3  # linear force
    mujoco.mj_step(m, d, 100)  # get some dynamics going
    mujoco.mj_forward(m, d)

    mx = mjx.put_model(m)

    # fwd_actuation
    dx = jax.jit(mjx.fwd_actuation)(mx, mjx.put_data(m, d))
    _assert_attr_eq(d, dx, 'act_dot')
    _assert_attr_eq(d, dx, 'qfrc_actuator')

    # fwd_accleration (fwd_position and fwd_velocity already tested elsewhere)
    dx = jax.jit(mjx.fwd_acceleration)(mx, mjx.put_data(m, d))
    _assert_attr_eq(d, dx, 'qfrc_smooth')
    _assert_attr_eq(d, dx, 'qacc_smooth')

    # euler
    dx = jax.jit(mjx.euler)(mx, mjx.put_data(m, d))
    mujoco.mj_Euler(m, d)
    _assert_attr_eq(d, dx, 'act')
    _assert_attr_eq(d, dx, 'qpos')
    _assert_attr_eq(d, dx, 'time')

  def test_step(self):
    m = test_util.load_test_file('constraints.xml')
    d = mujoco.MjData(m)
    # apply some control and xfrc input
    d.ctrl = np.array([-18, 0.59, 0.47])
    d.xfrc_applied[0, 2] = 0.1  # torque
    d.xfrc_applied[1, 4] = 0.3  # linear force
    mujoco.mj_step(m, d, 100)  # get some dynamics going

    mx = mjx.put_model(m)
    dx = jax.jit(mjx.step)(mx, mjx.put_data(m, d))
    mujoco.mj_step(m, d)
    _assert_attr_eq(d, dx, 'act')
    _assert_attr_eq(d, dx, 'time')
    _assert_attr_eq(d, dx, 'qvel')
    _assert_attr_eq(d, dx, 'qpos')

  def test_rk4(self):
    m = mujoco.MjModel.from_xml_string("""
        <mujoco>
          <option integrator="RK4">
            <flag constraint="disable"/>
          </option>
          <worldbody>
            <geom type="plane" size="1 1 .01" pos="0 0 -1"/>
            <body pos="0.15 0 0">
              <joint type="hinge" axis="0 1 0"/>
              <geom type="capsule" size="0.02" fromto="0 0 0 .1 0 0"/>
              <body pos="0.1 0 0">
                <joint type="slide" axis="1 0 0" stiffness="200"/>
                <geom type="capsule" size="0.015" fromto="-.1 0 0 .1 0 0"/>
              </body>
            </body>
          </worldbody>
        </mujoco>
        """)

    d = mujoco.MjData(m)
    # give the system a little kick to ensure we have non-identity rotations
    d.qvel = np.array([0.2, -0.1])
    mujoco.mj_step(m, d, 10)  # let dynamics get state significantly non-zero
    mujoco.mj_forward(m, d)

    mx = mjx.put_model(m)
    dx = jax.jit(mjx.rungekutta4)(mx, mjx.put_data(m, d))
    mujoco.mj_RungeKutta(m, d, 4)

    _assert_attr_eq(d, dx, 'qvel')
    _assert_attr_eq(d, dx, 'qpos')
    _assert_attr_eq(d, dx, 'act')
    _assert_attr_eq(d, dx, 'time')

  def test_disable_eulerdamp(self):
    m = test_util.load_test_file('pendula.xml')
    self.assertTrue((m.dof_damping > 0).any())
    m.opt.disableflags = m.opt.disableflags | mjx.DisableBit.EULERDAMP

    d = mujoco.MjData(m)
    d.qvel[:] = 1.0
    d.qacc[:] = 1.0
    mx = mjx.put_model(m)
    dx = jax.jit(mjx.euler)(mx, mjx.put_data(m, d))

    np.testing.assert_allclose(dx.qvel, 1 + m.opt.timestep)


class ActuatorTest(absltest.TestCase):
  _DYN_XML = """
    <mujoco>
      <compiler autolimits="true"/>
      <worldbody>
        <body name="box">
          <joint name="slide1" type="slide" axis="1 0 0" />
          <joint name="slide2" type="slide" axis="0 1 0" />
          <joint name="slide3" type="slide" axis="0 0 1" />
          <joint name="slide4" type="slide" axis="1 1 0" />
          <geom type="box" size=".05 .05 .05" mass="1"/>
        </body>
      </worldbody>
      <actuator>
        <general joint="slide1" dynprm="0.1" gainprm="1.1" />
        <general joint="slide2" dyntype="integrator" dynprm="0.1" gainprm="1.1" />
        <general joint="slide3" dyntype="filter" dynprm="0.1" gainprm="1.1" />
        <general joint="slide4" dyntype="filterexact" dynprm="0.1" gainprm="1.1" />
      </actuator>
    </mujoco>
  """

  def test_dyntype(self):
    m = mujoco.MjModel.from_xml_string(self._DYN_XML)
    d = mujoco.MjData(m)
    d.ctrl = np.array([1.5, 1.5, 1.5, 1.5])
    d.act = np.array([0.5, 0.5, 0.5])

    mx = mjx.put_model(m)
    dx = mjx.put_data(m, d)

    mujoco.mj_fwdActuation(m, d)
    dx = jax.jit(mjx.fwd_actuation)(mx, dx)
    _assert_attr_eq(d, dx, 'act_dot')

    mujoco.mj_Euler(m, d)
    dx = jax.jit(mjx.euler)(mx, dx)
    _assert_attr_eq(d, dx, 'act')


if __name__ == '__main__':
  absltest.main()
