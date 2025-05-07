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
"""Tests for inverse dynamics functions."""
from absl.testing import absltest
from absl.testing import parameterized
from jax import numpy as jp
import mujoco
from mujoco import mjx
from mujoco.mjx._src import support
import numpy as np

# tolerance for difference between MuJoCo and MJX calculations - mostly
# due to float precision
_TOLERANCE = 1e-5


def _assert_eq(a, b, name, tol=_TOLERANCE):
  tol = tol * 10  # avoid test noise
  err_msg = f'mismatch: {name}'
  np.testing.assert_allclose(a, b, err_msg=err_msg, atol=tol, rtol=tol)


class InverseTest(parameterized.TestCase):

  @parameterized.parameters(
      (mujoco.mjtIntegrator.mjINT_EULER, False, False),
      (mujoco.mjtIntegrator.mjINT_EULER, False, True),
      (mujoco.mjtIntegrator.mjINT_EULER, True, False),
      (mujoco.mjtIntegrator.mjINT_EULER, True, True),
      (mujoco.mjtIntegrator.mjINT_IMPLICITFAST, False, False),
      (mujoco.mjtIntegrator.mjINT_IMPLICITFAST, True, False),
  )
  def test_forward_inverse_match(self, integrator, invdiscrete, eulerdamp):
    m = mujoco.MjModel.from_xml_string("""
      <mujoco>
        <option timestep=".005" gravity="-1 -1 -10"/>
        <worldbody>
          <geom type="plane" size="10 10 .001"/>
          <body pos="0 0 1">
            <geom type="sphere" size=".1" pos=".1 .2 .3"/>
            <joint name="jnt1" type="hinge" axis="0 1 0" stiffness=".25" damping=".125"/>
            <body pos="0 0 1">
              <geom type="sphere" size=".1" pos=".1 .2 .3"/>
              <joint name="jnt2" type="hinge" axis="0 1 0" stiffness=".6" damping=".3"/>
            </body>
          </body>
        </worldbody>
        <actuator>
          <motor joint="jnt1"/>
        </actuator>
        <equality>
          <joint joint1="jnt1" joint2="jnt2"/>
        </equality>
      </mujoco>
    """)
    m.opt.integrator = integrator
    if invdiscrete:
      m.opt.enableflags |= mujoco.mjtEnableBit.mjENBL_INVDISCRETE
    if not eulerdamp:
      m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_EULERDAMP

    d = mujoco.MjData(m)
    d.qvel = np.random.uniform(low=-0.01, high=0.01, size=d.qvel.shape)
    d.ctrl = np.random.uniform(low=-0.01, high=0.01, size=d.ctrl.shape)
    d.qfrc_applied = np.random.uniform(
        low=-0.01, high=0.01, size=d.qfrc_applied.shape
    )
    d.xfrc_applied = np.random.uniform(
        low=-0.01, high=0.01, size=d.xfrc_applied.shape
    )
    mujoco.mj_step(m, d, 100)

    mx = mjx.put_model(m)
    dx = mjx.put_data(m, d)
    dx_next = mjx.step(mx, dx)
    qacc_fd = (dx_next.qvel - dx.qvel) / mx.opt.timestep

    dx = mjx.forward(mx, dx)

    if invdiscrete:
      dx = dx.replace(qacc=qacc_fd)

    dxinv = mjx.inverse(mx, dx)

    fwdinv0 = jp.linalg.norm(
        dxinv.qfrc_constraint - dx.qfrc_constraint, ord=np.inf
    )
    fwdinv1 = jp.linalg.norm(
        dxinv.qfrc_inverse
        - (
            dx.qfrc_applied + dx.qfrc_actuator + support.xfrc_accumulate(mx, dx)
        ),
        ord=np.inf,
    )

    self.assertLess(fwdinv0, 1.0e-3)
    self.assertLess(fwdinv1, 1.0e-3)
    _assert_eq(dxinv.qacc, dx.qacc, 'qacc')


if __name__ == '__main__':
  absltest.main()
