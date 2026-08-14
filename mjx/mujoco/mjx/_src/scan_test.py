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
"""Tests for scan functions."""

from absl.testing import absltest
from jax import numpy as jp
import mujoco
from mujoco import mjx
# pylint: disable=g-importing-member
from mujoco.mjx._src import scan
from mujoco.mjx._src.types import JointType
# pylint: enable=g-importing-member
import numpy as np


class ScanTest(absltest.TestCase):

  _MULTI_DOF_XML = """
      <mujoco>
        <compiler inertiafromgeom="true"/>
        <worldbody>
          <body>
            <joint type="free"/>
            <geom size=".15" mass="1" type="sphere"/>
            <body>
              <joint axis="1 0 0" pos="1 0 0" type="ball"/>
              <geom size=".15" mass="2" type="sphere"/>
            </body>
            <body>
              <joint axis="1 0 0" pos="2 0 0" type="hinge"/>
              <joint axis="0 1 0" pos="3 0 0" type="slide"/>
              <geom size=".15" mass="3" type="sphere"/>
            </body>
          </body>
        </worldbody>
      </mujoco>
    """

  def test_flat_empty(self):
    """Test scanning over just world body."""
    m = mujoco.MjModel.from_xml_string("""
      <mujoco model="world_body">
        <worldbody/>
      </mujoco>
    """)
    m = mjx.put_model(m)

    def fn(body_id):
      return body_id + 1

    b_in = jp.array([1])
    self.assertRaises(ValueError, scan.flat, m, fn, 'b', 'b', b_in)

  def test_flat_joints(self):
    """Tests scanning over bodies with joints of different types."""
    m = mujoco.MjModel.from_xml_string(self._MULTI_DOF_XML)
    m = mjx.put_model(m)

    # we will test two functions:
    #   1) j_fn receives jnt_types as a jp array
    #   2) s_fn receives jnt_types as a static np array and can switch on it
    j_fn = lambda jnt_pos, val: val + jp.sum(jnt_pos)
    s_fn = lambda jnt_types, val: val + sum(jnt_types)

    b_in = jp.array([[0, 0], [1, 1], [2, 2], [3, 3]])
    b_expect = jp.array([[0, 0], [1, 1], [3, 3], [8, 8]])
    b_out = scan.flat(m, j_fn, 'jb', 'b', m.jnt_pos, b_in)
    np.testing.assert_equal(np.array(b_out), np.array(b_expect))

    b_out = scan.flat(m, s_fn, 'jb', 'b', m.jnt_type, b_in)
    np.testing.assert_equal(np.array(b_out), np.array(b_expect))

    # None should be omitted from the results
    def no_free(jnt_types, val):
      if tuple(jnt_types) == (JointType.FREE,):
        return None
      return val + sum(jnt_types)

    b_expect = jp.array([[0, 0], [3, 3], [8, 8]])
    b_out = scan.flat(m, no_free, 'jb', 'b', m.jnt_type, b_in)
    np.testing.assert_equal(np.array(b_out), np.array(b_expect))

    # we should not call functions for which we know we will discard the results
    def no_world(jnt_types, val):
      if jnt_types.size == 0:
        self.fail('world has no dofs, should not be called')
      return val + sum(jnt_types)

    v_in = jp.ones((m.nv, 1))
    scan.flat(m, no_world, 'jv', 'v', m.jnt_type, v_in)

  def test_body_tree(self):
    """Tests tree scanning over bodies with different joint counts."""
    m = mujoco.MjModel.from_xml_string(self._MULTI_DOF_XML)
    m = mjx.put_model(m)

    # we will test two functions:
    #   1) j_fn receives jnt_pos which is a jp array
    #   2) s_fn receives jnt_types which is a static np array
    def j_fn(carry, jnt_pos, val):
      carry = jp.zeros_like(val) if carry is None else carry
      return carry + val + jp.sum(jnt_pos)

    def s_fn(carry, jnt_types, val):
      carry = jp.zeros_like(val) if carry is None else carry
      return carry + val + sum(jnt_types)

    b_in = jp.array([[0, 0], [1, 1], [2, 2], [3, 3]])
    b_expect = jp.array([[0, 0], [1, 1], [4, 4], [9, 9]])

    b_out = scan.body_tree(m, j_fn, 'jb', 'b', m.jnt_pos, b_in)
    np.testing.assert_equal(np.array(b_out), np.array(b_expect))

    b_out = scan.body_tree(m, s_fn, 'jb', 'b', m.jnt_type, b_in)
    np.testing.assert_equal(np.array(b_out), np.array(b_expect))

    # and reverse too:
    b_expect = jp.array([[12, 12], [12, 12], [3, 3], [8, 8]])
    b_out = scan.body_tree(m, j_fn, 'jb', 'b', m.jnt_pos, b_in, reverse=True)
    np.testing.assert_equal(np.array(b_out), np.array(b_expect))

    b_out = scan.body_tree(m, s_fn, 'jb', 'b', m.jnt_type, b_in, reverse=True)
    np.testing.assert_equal(np.array(b_out), np.array(b_expect))

    # None should be omitted from the results
    def no_free(carry, jnt_types, val):
      if tuple(jnt_types) == (JointType.FREE,):
        return None
      carry = jp.zeros_like(val) if carry is None else carry
      return carry + val + sum(jnt_types)

    b_expect = jp.array([[0, 0], [3, 3], [8, 8]])
    b_out = scan.body_tree(m, no_free, 'jb', 'b', m.jnt_type, b_in)
    np.testing.assert_equal(np.array(b_out), np.array(b_expect))

  _MULTI_ACT_XML = """
    <mujoco>
      <option timestep="0.02"/>
      <compiler autolimits="true"/>
      <default>
        <geom contype="0" conaffinity="0"/>
      </default>
      <worldbody>
        <body pos="0 0 -0.5">
          <joint type="free" name="joint0" range="-37.81 86.15"/>
          <geom pos="0 0.5 0" size=".15" mass="1" type="sphere"/>
          <body pos="0 0 -0.5">
            <joint axis="1 0 0" type="hinge" name="joint1" range="-32.88 5.15" actuatorfrcrange="-0.48 0.72"/>
            <geom pos="0 0.5 0" size=".15" mass="1" type="sphere"/>
            <body pos="0 0 -0.5">
              <joint axis="0 1 0" type="slide" name="joint2" range="-83.31 54.77"/>
              <joint axis="1 0 0" type="slide" name="joint3" range="-87.84 60.58" actuatorfrcrange="-0.62 0.28"/>
              <joint axis="0 1 0" type="hinge" name="joint4" range="-49.96 83.03" actuatorfrcrange="-0.01 0.64"/>
              <geom pos="0 0.5 0" size=".15" mass="1" type="sphere"/>
              <body pos="0 0 -0.5">
                <joint axis="1 0 0" type="slide" name="joint5" range="-1.53 87.07" actuatorfrcrange="-0.06 0.26"/>
                <joint axis="0 1 0" type="slide" name="joint6" range="-63.57 9.95"/>
                <joint axis="1 0 0" type="hinge" name="joint7" range="-22.50 41.20" actuatorfrcrange="-0.41 0.84"/>
                <geom pos="0 0.5 0" size=".15" mass="1" type="sphere"/>
                <body pos="0 0 -0.5">
                  <joint axis="0 1 0" type="hinge" name="joint8" range="-34.67 72.56"/>
                  <joint axis="1 0 0" type="hinge" name="joint9" range="-1.81 16.22" actuatorfrcrange="-0.41 0.14"/>
                  <joint axis="1 0 0" type="ball" name="joint10" range="0.00 6.02" actuatorfrcrange="-0.74 0.12"/>
                  <geom pos="0 0.5 0" size=".15" mass="1" type="sphere"/>
                </body>
              </body>
            </body>
          </body>
        </body>
      </worldbody>
      <actuator>
        <position joint="joint6" gear="1"/>
        <intvelocity joint="joint1" kp="2000" actrange="-0.7 2.3"/>
        <motor joint="joint3" gear="42"/>
        <velocity joint="joint2" kv="123"/>
        <position joint="joint0" ctrlrange="-0.9472 0.9472"/>
        <intvelocity joint="joint7" kp="2000" actrange="-0.7 2.3"/>
        <general joint="joint5" ctrlrange="0.1 2.34346" biastype="affine" gainprm="35 0 0" biasprm="0 -35 -0.65"/>
        <general joint="joint4" ctrlrange="0.1 2.34346" biastype="affine" gainprm="35 0 0" biasprm="0 -35 -0.65"/>
      </actuator>
    </mujoco>
  """

  def test_scan_actuators(self):
    """Tests scanning over actuators."""
    m = mujoco.MjModel.from_xml_string(self._MULTI_ACT_XML)
    m = mjx.put_model(m)

    fn = lambda *args: args
    args = (
        m.actuator_gear,
        m.jnt_type,
        jp.arange(m.nq),
        jp.arange(m.nv),
        jp.array([1.4, 1.1]),
    )
    gear, jnt_typ, qadr, vadr, act = scan.flat(
        m, fn, 'ujqva', 'ujqva', *args, group_by='u'
    )

    actuator_trnid = m.actuator_trnid[:, 0]
    np.testing.assert_array_equal(gear, m.actuator_gear)
    np.testing.assert_array_equal(jnt_typ, m.jnt_type[actuator_trnid])
    np.testing.assert_array_equal(act, jp.array([1.4, 1.1]))
    expected_vadr = np.concatenate(
        [np.nonzero(m.dof_jntid == trnid)[0] for trnid in actuator_trnid]
    )
    np.testing.assert_array_equal(vadr, expected_vadr)
    expected_qadr = np.concatenate(
        [np.nonzero(scan._q_jointid(m) == i)[0] for i in actuator_trnid]
    )
    np.testing.assert_array_equal(qadr, expected_qadr)


if __name__ == '__main__':
  absltest.main()
