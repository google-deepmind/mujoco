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

"""Unrolls trajectories within a scene and check for some coarse expected final state.

These tests are meant to catch subtle numerical / physics stability regressions.  MuJoCo has many
complex algorithms that interact in surprising ways.  Some physics bugs only arise over long
horizons of hundreds or thousands of physics steps.  These tests ought to catch some of those
regressions.
"""

import mujoco
import numpy as np
import warp as wp
from absl.testing import absltest
from absl.testing import parameterized

import mujoco_warp as mjw
from mujoco.mjx.third_party.mujoco_warp import ConeType
from mujoco.mjx.third_party.mujoco_warp import test_data

from mujoco.mjx.third_party.mujoco_warp._src.io import find_keys
from mujoco.mjx.third_party.mujoco_warp._src.io import make_trajectory


class UnrollTest(parameterized.TestCase):
  @absltest.skipIf(not wp.get_device().is_cuda, "Skipping test that requires GPU.")
  @parameterized.parameters(ConeType.PYRAMIDAL, ConeType.ELLIPTIC)
  def test_aloha_lifts_pot(self, cone):
    """Aloha lifts a pot into the air."""
    mjm, _, m, d = test_data.fixture("aloha_pot/scene.xml", keyframe="lift_pot0", overrides={"opt.cone": cone})

    with wp.ScopedCapture() as capture:
      mjw.step(m, d)

    for ctrl in make_trajectory(mjm, find_keys(mjm, "lift_pot")):
      wp.copy(d.ctrl, wp.array([ctrl.astype(np.float32)]))
      wp.capture_launch(capture.graph)

    # pot should be up in the air, lid should be open above it
    pot = mujoco.mj_name2id(mjm, mujoco.mjtObj.mjOBJ_BODY, "partnet_100015")
    lid = mujoco.mj_name2id(mjm, mujoco.mjtObj.mjOBJ_BODY, "partnet_100015/link_0")

    self.assertGreater(d.xpos.numpy()[0, pot, 2], 0.069)
    self.assertGreater(d.xpos.numpy()[0, lid, 2], 0.16)


if __name__ == "__main__":
  wp.init()
  absltest.main()
