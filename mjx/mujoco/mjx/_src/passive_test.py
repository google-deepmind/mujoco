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
"""Tests passive forces."""

from absl.testing import absltest
import jax
import mujoco
from mujoco import mjx
from mujoco.mjx._src import test_util
from jax import numpy as jp
import numpy as np

# tolerance for difference between MuJoCo and MJX passive calculations - mostly
# due to float precision
_TOLERANCE = 1e-7


def _assert_eq(a, b, name):
  tol = _TOLERANCE * 10  # avoid test noise
  err_msg = f'mismatch: {name}'
  np.testing.assert_allclose(a, b, err_msg=err_msg, atol=tol, rtol=tol)


def _assert_attr_eq(a, b, attr):
  _assert_eq(getattr(a, attr), getattr(b, attr), attr)


class PassiveTest(absltest.TestCase):

  def test_passive(self):
    m = test_util.load_test_file('pendula.xml')
    d = mujoco.MjData(m)
    # give the system a little kick to ensure we have non-identity rotations
    d.ctrl = np.array(
        [0.1, -0.1, 0.2, 0.3, -0.4, 0.5, -0.6, 0.1, -0.2, 0.1, 0.2]
    )
    mujoco.mj_step(m, d, 10)  # let dynamics get state significantly non-zero
    mujoco.mj_forward(m, d)
    mx = mjx.put_model(m)

    dx = jax.jit(mjx.passive)(mx, mjx.put_data(m, d))
    _assert_attr_eq(d, dx, 'qfrc_passive')
    _assert_attr_eq(d, dx, 'qfrc_gravcomp')

    # test with fluid forces
    m.opt.density = 0.01
    mujoco.mj_forward(m, d)
    mx = mjx.put_model(m)
    dx = jax.jit(mjx.passive)(mx, mjx.put_data(m, d))
    _assert_attr_eq(d, dx, 'qfrc_passive')
    _assert_attr_eq(d, dx, 'qfrc_gravcomp')

    m.opt.viscosity = 0.02
    mujoco.mj_forward(m, d)
    mx = mjx.put_model(m)
    dx = jax.jit(mjx.passive)(mx, mjx.put_data(m, d))
    _assert_attr_eq(d, dx, 'qfrc_passive')
    _assert_attr_eq(d, dx, 'qfrc_gravcomp')

    m.opt.wind = np.array([0.03, 0.04, 0.05])
    mujoco.mj_forward(m, d)
    mx = mjx.put_model(m)
    dx = jax.jit(mjx.passive)(mx, mjx.put_data(m, d))
    _assert_attr_eq(d, dx, 'qfrc_passive')
    _assert_attr_eq(d, dx, 'qfrc_gravcomp')

    # test disable passive
    mx = mx.tree_replace(
        {'opt.disableflags': mjx.DisableBit.SPRING | mjx.DisableBit.DAMPER}
    )
    dx = jax.jit(mjx.passive)(mx, mjx.put_data(m, d))
    np.testing.assert_allclose(dx.qfrc_passive, 0)

  def test_disable_spring_keeps_damper(self):
    """Disabling only springs must leave damping forces intact."""
    m = mujoco.MjModel.from_xml_string("""
      <mujoco>
        <option><flag spring="disable"/></option>
        <worldbody>
          <body>
            <joint name="j" damping="2"/>
            <geom size=".1"/>
          </body>
        </worldbody>
      </mujoco>
    """)
    d = mujoco.MjData(m)
    d.qvel[0] = 1.0
    mujoco.mj_forward(m, d)

    mx = mjx.put_model(m)
    dx = jax.jit(mjx.forward)(mx, mjx.make_data(mx).replace(qvel=jp.array([1.0])))

    _assert_eq(d.qfrc_passive, dx.qfrc_passive, 'qfrc_passive')


if __name__ == '__main__':
  absltest.main()
