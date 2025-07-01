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
from absl.testing import parameterized
import jax
from jax import numpy as jp
import mujoco
from mujoco import mjx
from mujoco.mjx._src import test_util
import numpy as np


# tolerance for difference between MuJoCo and MJX forward calculations - mostly
# due to float precision
_TOLERANCE = 1e-5


def _assert_eq(a, b, name, tol=_TOLERANCE):
  tol = tol * 10  # avoid test noise
  err_msg = f'mismatch: {name}'
  np.testing.assert_allclose(a, b, err_msg=err_msg, atol=tol, rtol=tol)


def _assert_attr_eq(a, b, attr, tol=_TOLERANCE):
  _assert_eq(getattr(a, attr), getattr(b, attr), attr, tol=tol)


class ForwardTest(absltest.TestCase):

  def test_forward(self):
    m = test_util.load_test_file('constraints.xml')
    d = mujoco.MjData(m)
    # apply some control and xfrc input
    d.ctrl = np.array([-18, 0.59, 0.47])
    d.xfrc_applied[0, 2] = 0.1  # torque
    d.xfrc_applied[1, 4] = 0.3  # linear force
    mujoco.mj_step(m, d, 20)  # get some dynamics going
    mujoco.mj_forward(m, d)

    mx = mjx.put_model(m)

    # fwd_actuation
    dx = mjx.put_data(m, d).replace(
        act_dot=np.zeros_like(d.act_dot),
        qfrc_actuator=np.zeros_like(d.qfrc_actuator),
        actuator_force=np.zeros_like(d.actuator_force),
    )
    dx = jax.jit(mjx.fwd_actuation)(mx, dx)
    _assert_attr_eq(d, dx, 'act_dot')
    _assert_attr_eq(d, dx, 'qfrc_actuator')
    _assert_attr_eq(d, dx, 'actuator_force')

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

    # implicitfast
    m.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    dx = jax.jit(mjx.implicit)(mx, mjx.put_data(m, d))
    mujoco.mj_implicit(m, d)
    _assert_attr_eq(d, dx, 'qpos')

  def test_step(self):
    m = test_util.load_test_file('constraints.xml')
    d = mujoco.MjData(m)
    # apply some control and xfrc input
    d.ctrl = np.array([-18, 0.59, 0.47])
    d.xfrc_applied[0, 2] = 0.1  # torque
    d.xfrc_applied[1, 4] = 0.3  # linear force
    mujoco.mj_step(m, d, 20)  # get some dynamics going

    dx = jax.jit(mjx.step)(mjx.put_model(m), mjx.put_data(m, d))
    mujoco.mj_step(m, d)
    _assert_attr_eq(d, dx, 'act')
    _assert_attr_eq(d, dx, 'time')
    _assert_attr_eq(d, dx, 'qvel', tol=5e-4)
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

    dx = jax.jit(mjx.rungekutta4)(mjx.put_model(m), mjx.put_data(m, d))
    mujoco.mj_RungeKutta(m, d, 4)

    _assert_attr_eq(d, dx, 'qvel')
    _assert_attr_eq(d, dx, 'qpos')
    _assert_attr_eq(d, dx, 'act')
    _assert_attr_eq(d, dx, 'time')
    _assert_attr_eq(d, dx, 'xpos')

  def test_eulerdamp(self):
    m = test_util.load_test_file('pendula.xml')
    self.assertTrue((m.dof_damping > 0).any())

    d = mujoco.MjData(m)
    d.qvel[:] = 1.0
    d.qacc[:] = 1.0
    mujoco.mj_forward(m, d)
    dx = jax.jit(mjx.euler)(mjx.put_model(m), mjx.put_data(m, d))
    mujoco.mj_Euler(m, d)

    _assert_attr_eq(d, dx, 'qpos')

    # also test sparse
    m.opt.jacobian = mujoco.mjtJacobian.mjJAC_SPARSE
    d = mujoco.MjData(m)
    d.qvel[:] = 1.0
    d.qacc[:] = 1.0
    mujoco.mj_forward(m, d)
    dx = jax.jit(mjx.euler)(mjx.put_model(m), mjx.put_data(m, d))
    mujoco.mj_Euler(m, d)

    _assert_attr_eq(d, dx, 'qpos')

  def test_disable_eulerdamp(self):
    m = test_util.load_test_file('pendula.xml')
    self.assertTrue((m.dof_damping > 0).any())
    m.opt.disableflags = m.opt.disableflags | mjx.DisableBit.EULERDAMP

    d = mujoco.MjData(m)
    d.qvel[:] = 1.0
    d.qacc[:] = 1.0
    dx = jax.jit(mjx.euler)(mjx.put_model(m), mjx.put_data(m, d))

    np.testing.assert_allclose(dx.qvel, 1 + m.opt.timestep)


class ActuatorTest(parameterized.TestCase):

  @parameterized.parameters(
      'actuator/arm21.xml',
      'actuator/arm26.xml',
      'actuator/general_dyntype.xml',
  )
  def test_actuator(self, fname):
    m = test_util.load_test_file(fname)
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d)
    d.ctrl = 1.5 * np.random.random(m.nu)
    d.act = 0.5 * np.random.random(m.na)
    mx = mjx.put_model(m)
    dx = mjx.put_data(m, d)

    mujoco.mj_fwdActuation(m, d)
    dx = jax.jit(mjx.fwd_actuation)(mx, dx)

    _assert_attr_eq(d, dx, 'act_dot')
    _assert_attr_eq(d, dx, 'qfrc_actuator')
    _assert_attr_eq(d, dx, 'actuator_force')

    mujoco.mj_Euler(m, d)
    dx = jax.jit(mjx.euler)(mx, dx)
    _assert_attr_eq(d, dx, 'act')

  def test_tendon_force_clamp(self):
    m = test_util.load_test_file('actuator/tendon_force_clamp.xml')
    d = mujoco.MjData(m)
    mx = mjx.put_model(m)
    dx = mjx.put_data(m, d)

    dx = dx.replace(ctrl=jp.array([1.0, 1.0, 1.0, -4.0, 1.0, -20.0, 5.0, -5.0]))
    dx = mjx.forward(mx, dx)

    _assert_eq(
        dx.actuator_force,
        jp.array([1.0, 1.0, 1.0, -4.0 / 3.0, 1.0 / 3.0, -10.0, 5.0, -5.0]),
        'actuator_force',
    )

    _assert_eq(
        dx.sensordata,
        jp.array([3.0, -1.0, -10.0, 0.0]),
        'sensordata',
    )


if __name__ == '__main__':
  absltest.main()
