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

"""Tests for passive force functions."""

import numpy as np
import warp as wp
from absl.testing import absltest
from absl.testing import parameterized

import mujoco_warp as mjwarp

from mujoco.mjx.third_party.mujoco_warp._src import test_util

# tolerance for difference between MuJoCo and MJWarp passive force calculations - mostly
# due to float precision
_TOLERANCE = 5e-5


def _assert_eq(a, b, name):
  tol = _TOLERANCE * 10  # avoid test noise
  err_msg = f"mismatch: {name}"
  np.testing.assert_allclose(a, b, err_msg=err_msg, atol=tol, rtol=tol)


class PassiveTest(parameterized.TestCase):
  @parameterized.product(passive=[True, False], gravity=[True, False])
  def test_passive(self, passive, gravity):
    """Tests passive."""
    _, mjd, m, d = test_util.fixture("pendula.xml", spring=passive, damper=passive, gravity=gravity, kick=True, applied=True)

    for arr in (d.qfrc_spring, d.qfrc_damper, d.qfrc_gravcomp, d.qfrc_passive):
      arr.zero_()

    mjwarp.passive(m, d)

    _assert_eq(d.qfrc_spring.numpy()[0], mjd.qfrc_spring, "qfrc_spring")
    _assert_eq(d.qfrc_damper.numpy()[0], mjd.qfrc_damper, "qfrc_damper")
    _assert_eq(d.qfrc_gravcomp.numpy()[0], mjd.qfrc_gravcomp, "qfrc_gravcomp")
    _assert_eq(d.qfrc_passive.numpy()[0], mjd.qfrc_passive, "qfrc_passive")

  @parameterized.parameters(
    (1, 0, 0, 0, 0),
    (0, 1, 0, 0, 0),
    (0, 0, 1, 0, 0),
    (0, 0, 0, 1, 0),
    (0, 0, 0, 0, 1),
    (1, 1, 1, 1, 1),
  )
  def test_fluid(self, density, viscosity, wind0, wind1, wind2):
    """Tests fluid model."""

    _, mjd, m, d = test_util.fixture(
      xml=f"""
      <mujoco>
        <option density="{density}" viscosity="{viscosity}" wind="{wind0} {wind1} {wind2}"/>
        <worldbody>
          <body>
            <geom type="box" size=".1 .1 .1"/>
            <freejoint/>
          </body>
        </worldbody>
        <keyframe>
          <key qvel="1 1 1 1 1 1"/>
        </keyframe>
      </mujoco>
    """,
      keyframe=0,
    )

    for arr in (d.qfrc_passive, d.qfrc_fluid):
      arr.zero_()

    mjwarp.passive(m, d)

    _assert_eq(d.qfrc_passive.numpy()[0], mjd.qfrc_passive, "qfrc_passive")
    _assert_eq(d.qfrc_fluid.numpy()[0], mjd.qfrc_fluid, "qfrc_fluid")

  @parameterized.parameters((True, True), (True, False), (False, True), (False, False))
  def test_gravcomp(self, sparse, gravity):
    """Tests gravity compensation."""

    _, mjd, m, d = test_util.fixture(
      xml="""
      <mujoco>
        <option gravity="1 2 3">
          <flag contact="disable"/>
        </option>
        <worldbody>
          <body gravcomp="1">
            <geom type="sphere" size=".1" pos="1 0 0"/>
            <joint name="joint0" type="hinge" axis="0 1 0" actuatorgravcomp="true"/>
          </body>
          <body gravcomp="1">
            <geom type="sphere" size=".1"/>
            <joint name="joint1" type="hinge" axis="1 0 0"/>
            <joint type="hinge" axis="0 1 0"/>
            <joint type="hinge" axis="0 0 1"/>
          </body>
          <body gravcomp="1">
            <geom type="sphere" size=".1"/>
            <joint type="hinge" axis="0 1 0"/>
          </body>
          <body gravcomp="0">
            <geom type="sphere" size=".1"/>
            <joint type="hinge" axis="0 1 0"/>
          </body>
        </worldbody>
        <actuator>
          <motor joint="joint0"/>
          <motor joint="joint1"/>
        </actuator>
      </mujoco>
    """,
      gravity=gravity,
      sparse=sparse,
    )

    for arr in (d.qfrc_passive, d.qfrc_gravcomp, d.qfrc_actuator):
      arr.zero_()

    mjwarp.passive(m, d)
    mjwarp.fwd_actuation(m, d)

    _assert_eq(d.qfrc_passive.numpy()[0], mjd.qfrc_passive, "qfrc_passive")
    _assert_eq(d.qfrc_gravcomp.numpy()[0], mjd.qfrc_gravcomp, "qfrc_gravcomp")
    _assert_eq(d.qfrc_actuator.numpy()[0], mjd.qfrc_actuator, "qfrc_actuator")


if __name__ == "__main__":
  wp.init()
  absltest.main()
