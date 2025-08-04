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

"""Tests for inverse dynamics."""

import mujoco
import numpy as np
import warp as wp
from absl.testing import absltest
from absl.testing import parameterized

import mujoco_warp as mjwarp

from mujoco.mjx.third_party.mujoco_warp._src import inverse
from mujoco.mjx.third_party.mujoco_warp._src import support
from mujoco.mjx.third_party.mujoco_warp._src import test_util
from mujoco.mjx.third_party.mujoco_warp._src.types import IntegratorType


def _assert_eq(a, b, name):
  tol = 5e-3  # avoid test noise
  err_msg = f"mismatch: {name}"
  np.testing.assert_allclose(a, b, err_msg=err_msg, atol=tol, rtol=tol)


_XML = """
<mujoco>
  <option timestep=".01" gravity="-1 -1 -1"/>
  <worldbody>
    <body>
      <geom type="sphere" size=".1" pos=".5 0 0"/>
      <joint name="joint1" type="hinge" axis="0 1 0" damping=".1"/>
      <body>
        <geom type="sphere" size=".2" pos="1 0 0"/>
        <joint name="joint2" type="hinge" axis="0 1 0" damping=".2"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor joint="joint1"/>
  </actuator>
  <equality>
    <joint joint1="joint1" joint2="joint2"/>
  </equality>
</mujoco>
"""


class InverseTest(parameterized.TestCase):
  @parameterized.product(
    integrator=[IntegratorType.EULER, IntegratorType.IMPLICITFAST],
    invdiscrete=[True, False],
    sparse=[True, False],
  )
  def test_inverse(self, integrator, invdiscrete, sparse):
    """Tests inverse dynamics."""
    mjm, mjd, m, d = test_util.fixture(
      xml=_XML,
      contact=False,
      integrator=integrator,
      kick=True,
      applied=True,
      nstep=10,
      sparse=sparse,
    )

    # discrete qacc
    if invdiscrete:
      mjm.opt.enableflags |= mujoco.mjtEnableBit.mjENBL_INVDISCRETE

      # save state
      qpos = mjd.qpos.copy()
      qvel = mjd.qvel.copy()

      # call step, save new qvel
      mujoco.mj_step(mjm, mjd)
      qvel_next = mjd.qvel.copy()

      # reset the state, compute discrete-time (finite-differenced) qacc
      mjd.qpos = qpos
      mjd.qvel = qvel
      qacc_fd = (qvel_next - qvel) / mjm.opt.timestep

      # call forward, overwrite qacc with qacc_fd
      mujoco.mj_forward(mjm, mjd)
      mjd.qacc = qacc_fd

      m = mjwarp.put_model(mjm)
      d = mjwarp.put_data(mjm, mjd)

    qacc = d.qacc.numpy()[0].copy()
    qfrc_constraint = d.qfrc_constraint.numpy()[0].copy()

    # qfrc_inverse = qfrc_applied + J.T @ xfrc_applied + qfrc_actuator
    qfrc_xfrc_applied = wp.zeros((d.nworld, m.nv), dtype=float)
    support.xfrc_accumulate(m, d, qfrc_xfrc_applied)
    qfrc_inverse = d.qfrc_applied.numpy()[0] + d.qfrc_actuator.numpy()[0] + qfrc_xfrc_applied.numpy()[0]

    for arr in (d.qfrc_constraint, d.qfrc_inverse):
      arr.zero_()

    mjwarp.inverse(m, d)

    _assert_eq(d.qfrc_constraint.numpy()[0], qfrc_constraint, "qfrc_constraint")
    _assert_eq(d.qfrc_inverse.numpy()[0], qfrc_inverse, "qfrc_inverse")
    _assert_eq(d.qacc.numpy()[0], qacc, "qacc")

  def test_discrete_acc_eulerdamp(self):
    _, _, m, d = test_util.fixture(
      xml=_XML, integrator=IntegratorType.EULER, eulerdamp=False, kick=True, applied=True, nstep=10
    )
    qacc = wp.zeros((1, m.nv), dtype=float)
    qfrc = wp.zeros((1, m.nv), dtype=float)
    inverse.discrete_acc(m, d, qacc, qfrc)
    _assert_eq(qacc.numpy()[0], d.qacc.numpy()[0], "qacc")

  def test_discrete_acc_rk4(self):
    _, _, m, d = test_util.fixture(xml=_XML, integrator=IntegratorType.RK4)
    qacc = wp.zeros((1, m.nv), dtype=float)
    qfrc = wp.zeros((1, m.nv), dtype=float)

    with self.assertRaises(NotImplementedError):
      inverse.discrete_acc(m, d, qacc, qfrc)

  def test_inverse_tendon_armature(self):
    """Tests inverse dynamics with tendon armature."""
    _, _, m, d = test_util.fixture(
      "tendon/armature.xml",
      constraint=False,
      gravity=False,
      kick=True,
      applied=True,
      nstep=10,
      keyframe=0,
    )

    qacc = d.qacc.numpy()[0].copy()
    qfrc_constraint = d.qfrc_constraint.numpy()[0].copy()

    # qfrc_inverse = qfrc_applied + J.T @ xfrc_applied + qfrc_actuator
    qfrc_xfrc_applied = wp.zeros((d.nworld, m.nv), dtype=float)
    support.xfrc_accumulate(m, d, qfrc_xfrc_applied)
    qfrc_inverse = d.qfrc_applied.numpy()[0] + d.qfrc_actuator.numpy()[0] + qfrc_xfrc_applied.numpy()[0]

    for arr in (d.qfrc_constraint, d.qfrc_inverse):
      arr.zero_()

    mjwarp.inverse(m, d)

    _assert_eq(d.qfrc_constraint.numpy()[0], qfrc_constraint, "qfrc_constraint")
    _assert_eq(d.qfrc_inverse.numpy()[0], qfrc_inverse, "qfrc_inverse")
    _assert_eq(d.qacc.numpy()[0], qacc, "qacc")


if __name__ == "__main__":
  wp.init()
  absltest.main()
