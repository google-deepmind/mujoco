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
import mujoco
from mujoco import mjx
from mujoco.mjx._src import test_util
import numpy as np


def _assert_attr_eq(a, b, attr, step, fname, atol=1e-3, rtol=1e-3):
  err_msg = f'mismatch: {attr} at step {step} in {fname}'
  a, b = getattr(a, attr), getattr(b, attr)
  np.testing.assert_allclose(a, b, err_msg=err_msg, atol=atol, rtol=rtol)


class ActuationIntegrationTest(parameterized.TestCase):

  @parameterized.parameters(list(range(30)))
  def test_actuation(self, seed):
    mjcf = test_util.create_mjcf(
        seed,
        max_trees=1,
        max_tree_depth=5,
        body_pos=(0.0, 0.0, -0.5),
        geom_pos=(0.0, 0.5, 0.0),
        disable_actuation_pct=10,
        add_actuators=True,
        max_contact_excludes=2,
        enable_contact=False,
    )
    m = mujoco.MjModel.from_xml_string(mjcf)
    actuation_jit_fn = jax.jit(mjx.fwd_actuation)

    # init
    d = mujoco.MjData(m)
    d.ctrl = np.random.normal(scale=10, size=m.nu)
    d.act = np.random.normal(scale=10, size=m.na)
    d.qvel = np.random.random(m.nv)
    mujoco.mj_fwdPosition(m, d)
    mujoco.mj_fwdVelocity(m, d)

    # put on device
    mx = mjx.put_model(m)
    dx = mjx.put_data(m, d)

    mujoco.mj_fwdActuation(m, d)
    dx = actuation_jit_fn(mx, dx)

    _assert_attr_eq(d, dx, 'qfrc_actuator', seed, f'actuator{seed}')
    _assert_attr_eq(d, dx, 'act_dot', seed, f'actuator{seed}')


if __name__ == '__main__':
  absltest.main()
