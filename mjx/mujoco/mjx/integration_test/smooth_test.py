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
"""Tests for smooth dynamics functions."""

from absl.testing import absltest
from absl.testing import parameterized
import jax
import mujoco
from mujoco import mjx
from mujoco.mjx._src import test_util
import numpy as np


def _assert_eq(a, b, name, step, fname, atol=1e-5, rtol=1e-5):
  err_msg = f'mismatch: {name} at step {step} in {fname}'
  np.testing.assert_allclose(a, b, err_msg=err_msg, atol=atol, rtol=rtol)


def _assert_attr_eq(a, b, attr, step, fname, atol=1e-5, rtol=1e-5):
  err_msg = f'mismatch: {attr} at step {step} in {fname}'
  a, b = getattr(a, attr), getattr(b, attr)
  np.testing.assert_allclose(a, b, err_msg=err_msg, atol=atol, rtol=rtol)


class TransmissionIntegrationTest(parameterized.TestCase):

  @parameterized.parameters(list(range(30)))
  def test_transmission(self, seed):
    """Tests mjx.transmission matches mj_transmission."""
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
    transmission_jit_fn = jax.jit(mjx.transmission)

    # init
    d = mujoco.MjData(m)
    d.ctrl = np.random.normal(scale=10, size=m.nu)
    d.act = np.random.normal(scale=10, size=m.na)
    d.qpos = np.random.normal(m.nq)
    d.qvel = np.random.random(m.nv)
    mujoco.mj_forward(m, d)

    # put on device
    mx = mjx.put_model(m)
    dx = mjx.put_data(m, d)

    mujoco.mj_transmission(m, d)
    dx = transmission_jit_fn(mx, dx)

    _assert_attr_eq(
        d, dx, 'actuator_length', seed, f'transmission{seed}', atol=1e-4
    )

    # convert sparse actuator_moment to dense representation
    moment = np.zeros((m.nu, m.nv))
    mujoco.mju_sparse2dense(
        moment,
        d.actuator_moment,
        d.moment_rownnz,
        d.moment_rowadr,
        d.moment_colind,
    )
    _assert_eq(
        moment,
        dx.actuator_moment,
        'actuator_moment',
        seed,
        f'transmission{seed}',
        atol=1e-4,
    )


if __name__ == '__main__':
  absltest.main()
