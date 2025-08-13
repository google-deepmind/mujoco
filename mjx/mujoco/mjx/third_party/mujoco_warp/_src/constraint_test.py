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

"""Tests for constraint functions."""

import mujoco
import numpy as np
import warp as wp
from absl.testing import absltest
from absl.testing import parameterized

import mujoco_warp as mjwarp

from mujoco.mjx.third_party.mujoco_warp._src import test_util
from mujoco.mjx.third_party.mujoco_warp._src.types import ConeType

# tolerance for difference between MuJoCo and MJWarp constraint calculations,
# mostly due to float precision
_TOLERANCE = 5e-5


def _assert_eq(a, b, name):
  tol = _TOLERANCE * 10  # avoid test noise
  err_msg = f"mismatch: {name}"
  np.testing.assert_allclose(a, b, err_msg=err_msg, atol=tol, rtol=tol)


def _assert_efc_eq(d, mjd, nefc, name):
  """Assert equality of efc fields after sorting both sides."""
  # Get the ordering indices based on efc_type, efc_pos, efc_vel, efc_aref, efc_d for MJWarp
  efc_type = d.efc.type.numpy()[0, :nefc]
  efc_pos = d.efc.pos.numpy()[0, :nefc]
  efc_vel = d.efc.vel.numpy()[0, :nefc]
  efc_aref = d.efc.aref.numpy()[0, :nefc]
  efc_d = d.efc.D.numpy()[0, :nefc]
  # Get the ordering indices based on efc_type, efc_pos, efc_vel, efc_aref, efc_d for MuJoCo
  mjd_efc_type = mjd.efc_type[:nefc]
  mjd_efc_pos = mjd.efc_pos[:nefc]
  mjd_efc_vel = mjd.efc_vel[:nefc]
  mjd_efc_aref = mjd.efc_aref[:nefc]
  mjd_efc_d = mjd.efc_D[:nefc]

  # Create sorting keys using lexsort (more efficient for multiple keys)
  d_sort_indices = np.lexsort((efc_pos, efc_type, efc_vel, efc_aref, efc_d))
  mjd_sort_indices = np.lexsort((mjd_efc_pos, mjd_efc_type, mjd_efc_vel, mjd_efc_aref, mjd_efc_d))

  # Sort MJWarp efc fields
  d_sorted = d.efc.J.numpy()[0, d_sort_indices, :].reshape(-1)

  # Sort MuJoCo efc fields
  # For J matrix, need to reshape to 2D, sort rows, then flatten
  nefc = len(mjd_sort_indices)
  nv = mjd.efc_J.shape[0] // nefc if nefc > 0 else 0
  if nv > 0:
    mjd_J_2d = mjd.efc_J.reshape(nefc, nv)
    mjd_sorted_J = mjd_J_2d[mjd_sort_indices].reshape(-1)
  else:
    mjd_sorted_J = mjd.efc_J

  mjd_sorted_D = mjd.efc_D[mjd_sort_indices]
  mjd_sorted_vel = mjd.efc_vel[mjd_sort_indices]
  mjd_sorted_aref = mjd.efc_aref[mjd_sort_indices]
  mjd_sorted_pos = mjd.efc_pos[mjd_sort_indices]
  mjd_sorted_margin = mjd.efc_margin[mjd_sort_indices]
  mjd_sorted_type = mjd.efc_type[mjd_sort_indices]

  # Compare sorted data
  _assert_eq(d_sorted, mjd_sorted_J, f"{name}_J")

  d_sorted = d.efc.D.numpy()[0, d_sort_indices]
  _assert_eq(d_sorted, mjd_sorted_D, f"{name}_D")

  d_sorted = d.efc.vel.numpy()[0, d_sort_indices]
  _assert_eq(d_sorted, mjd_sorted_vel, f"{name}_vel")

  d_sorted = d.efc.aref.numpy()[0, d_sort_indices]
  _assert_eq(d_sorted, mjd_sorted_aref, f"{name}_aref")

  d_sorted = d.efc.pos.numpy()[0, d_sort_indices]
  _assert_eq(d_sorted, mjd_sorted_pos, f"{name}_pos")

  d_sorted = d.efc.margin.numpy()[0, d_sort_indices]
  _assert_eq(d_sorted, mjd_sorted_margin, f"{name}_margin")

  d_sorted = d.efc.type.numpy()[0, d_sort_indices]
  _assert_eq(d_sorted, mjd_sorted_type, f"{name}_type")


class ConstraintTest(parameterized.TestCase):
  @parameterized.parameters(
    (ConeType.PYRAMIDAL, 1, 1),
    (ConeType.PYRAMIDAL, 1, 3),
    (ConeType.PYRAMIDAL, 1, 4),
    (ConeType.PYRAMIDAL, 1, 6),
    (ConeType.PYRAMIDAL, 3, 3),
    (ConeType.PYRAMIDAL, 3, 4),
    (ConeType.PYRAMIDAL, 3, 6),
    (ConeType.PYRAMIDAL, 4, 4),
    (ConeType.PYRAMIDAL, 4, 6),
    (ConeType.PYRAMIDAL, 6, 6),
    (ConeType.ELLIPTIC, 1, 1),
    (ConeType.ELLIPTIC, 1, 3),
    (ConeType.ELLIPTIC, 1, 4),
    (ConeType.ELLIPTIC, 1, 6),
    (ConeType.ELLIPTIC, 3, 3),
    (ConeType.ELLIPTIC, 3, 4),
    (ConeType.ELLIPTIC, 3, 6),
    (ConeType.ELLIPTIC, 4, 4),
    (ConeType.ELLIPTIC, 4, 6),
    (ConeType.ELLIPTIC, 6, 6),
  )
  def test_condim(self, cone, condim1, condim2):
    """Test condim."""
    xml = f"""
      <mujoco>
        <worldbody>
          <body>
            <geom type="sphere" size=".1" condim="{condim1}"/>
            <freejoint/>
          </body>
          <body>
            <geom type="sphere" size=".1" condim="{condim2}"/>
          </body>
          <body>
            <geom type="ellipsoid" size=".1 .1 .1" condim="{condim2}"/>
          </body>
        </worldbody>
        <keyframe>
          <key qpos=".10 .11 .12 .7071 .7071 0 0" />
        </keyframe>
      </mujoco>
    """

    _, mjd, m, d = test_util.fixture(xml=xml, cone=cone, keyframe=0)

    # fill with nan to check whether we are not reading uninitialized values
    for arr in (d.efc.J, d.efc.D, d.efc.aref, d.efc.pos, d.efc.margin):
      arr.fill_(wp.nan)

    mjwarp.make_constraint(m, d)

    _assert_eq(d.ncon.numpy()[0], mjd.ncon, "ncon")
    _assert_eq(d.efc.J.numpy()[0, : mjd.nefc, :].reshape(-1), mjd.efc_J, "efc_J")
    _assert_eq(d.efc.D.numpy()[0, : mjd.nefc], mjd.efc_D, "efc_D")
    _assert_eq(d.efc.aref.numpy()[0, : mjd.nefc], mjd.efc_aref, "efc_aref")
    _assert_eq(d.efc.pos.numpy()[0, : mjd.nefc], mjd.efc_pos, "efc_pos")
    _assert_eq(d.efc.margin.numpy()[0, : mjd.nefc], mjd.efc_margin, "efc_margin")

  @parameterized.parameters(
    mujoco.mjtCone.mjCONE_PYRAMIDAL,
    mujoco.mjtCone.mjCONE_ELLIPTIC,
  )
  def test_constraints(self, cone):
    """Test constraints."""
    for key in range(3):
      _, mjd, m, d = test_util.fixture("constraints.xml", sparse=False, cone=cone, keyframe=key)

      for arr in (d.ne, d.nefc, d.nf, d.nl, d.efc.type):
        arr.fill_(-1)
      for arr in (d.efc.J, d.efc.D, d.efc.vel, d.efc.aref, d.efc.pos, d.efc.margin):
        arr.fill_(wp.nan)

      mjwarp.make_constraint(m, d)

      _assert_eq(d.ne.numpy()[0], mjd.ne, "ne")
      _assert_eq(d.nefc.numpy()[0], mjd.nefc, "nefc")
      _assert_eq(d.nf.numpy()[0], mjd.nf, "nf")
      _assert_eq(d.nl.numpy()[0], mjd.nl, "nl")
      _assert_efc_eq(d, mjd, mjd.nefc, "efc")

  def test_limit_tendon(self):
    """Test limit tendon constraints."""
    for keyframe in range(-1, 1):
      _, mjd, m, d = test_util.fixture("tendon/tendon_limit.xml", sparse=False, keyframe=keyframe)

      for arr in (d.nefc, d.nl, d.efc.type):
        arr.fill_(-1)
      for arr in (d.efc.J, d.efc.D, d.efc.vel, d.efc.aref, d.efc.pos, d.efc.margin):
        arr.fill_(wp.nan)

      mjwarp.make_constraint(m, d)

      _assert_eq(d.nefc.numpy()[0], mjd.nefc, "nefc")
      _assert_eq(d.nl.numpy()[0], mjd.nl, "nl")
      _assert_efc_eq(d, mjd, mjd.nefc, "efc")

  def test_equality_tendon(self):
    """Test equality tendon constraints."""

    _, mjd, m, d = test_util.fixture(
      xml="""
      <mujoco>
        <option>
          <flag contact="disable"/>
        </option>
        <worldbody>
          <body>
            <geom type="sphere" size=".1"/>
            <joint name="joint0" type="hinge"/>
          </body>
          <body>
            <geom type="sphere" size=".1"/>
            <joint name="joint1" type="hinge"/>
          </body>
          <body>
            <geom type="sphere" size=".1"/>
            <joint name="joint2" type="hinge"/>
          </body>
        </worldbody>
        <tendon>
          <fixed name="tendon0">
            <joint joint="joint0" coef=".1"/>
          </fixed>
          <fixed name="tendon1">
            <joint joint="joint1" coef=".2"/>
          </fixed>
          <fixed name="tendon2">
            <joint joint="joint2" coef=".3"/>
          </fixed>
        </tendon>
        <equality>
          <tendon tendon1="tendon0" tendon2="tendon1" polycoef=".1 .2 .3 .4 .5"/>
          <tendon tendon1="tendon2" polycoef="-.1 0 0 0 0"/>
        </equality>
        <keyframe>
          <key qpos=".1 .2 .3"/>
        </keyframe>
      </mujoco>
    """,
      keyframe=0,
    )

    for arr in (d.nefc, d.ne, d.efc.type):
      arr.fill_(-1)
    for arr in (d.efc.J, d.efc.D, d.efc.vel, d.efc.aref, d.efc.pos, d.efc.margin):
      arr.fill_(wp.nan)

    mjwarp.make_constraint(m, d)

    _assert_eq(d.nefc.numpy()[0], mjd.nefc, "nefc")
    _assert_eq(d.ne.numpy()[0], mjd.ne, "ne")
    _assert_efc_eq(d, mjd, mjd.nefc, "efc")


if __name__ == "__main__":
  absltest.main()
