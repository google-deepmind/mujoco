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

# tolerance for difference between MuJoCo and MJX smooth calculations - mostly
# due to float precision
_TOLERANCE = 5e-5


def _assert_eq(a, b, name):
  tol = _TOLERANCE * 10  # avoid test noise
  err_msg = f'mismatch: {name}'
  np.testing.assert_allclose(a, b, err_msg=err_msg, atol=tol, rtol=tol)


def _assert_attr_eq(a, b, attr):
  _assert_eq(getattr(a, attr), getattr(b, attr), attr)


class SmoothTest(absltest.TestCase):

  def setUp(self):
    super().setUp()
    # although we already have generous padding of thresholds, it doesn't hurt
    # to also fix the seed to reduce test flakiness
    np.random.seed(0)

  def test_smooth(self):
    """Tests MJX smooth functions match MuJoCo smooth functions."""

    m = test_util.load_test_file('pendula.xml')
    # tell MJX to use sparse mass matrices:
    m.opt.jacobian = mujoco.mjtJacobian.mjJAC_SPARSE
    d = mujoco.MjData(m)
    # give the system a little kick to ensure we have non-identity rotations
    d.qvel = np.random.random(m.nv)
    mujoco.mj_step(m, d, 10)  # let dynamics get state significantly non-zero
    mujoco.mj_forward(m, d)
    mx = mjx.put_model(m)

    # kinematics
    dx = jax.jit(mjx.kinematics)(mx, mjx.put_data(m, d))
    _assert_attr_eq(d, dx, 'xanchor')
    _assert_attr_eq(d, dx, 'xaxis')
    _assert_attr_eq(d, dx, 'xpos')
    _assert_attr_eq(d, dx, 'xquat')
    _assert_eq(d.xmat.reshape((-1, 3, 3)), dx.xmat, 'xmat')
    _assert_attr_eq(d, dx, 'xipos')
    _assert_eq(d.ximat.reshape((-1, 3, 3)), dx.ximat, 'ximat')
    _assert_attr_eq(d, dx, 'geom_xpos')
    _assert_eq(d.geom_xmat.reshape((-1, 3, 3)), dx.geom_xmat, 'geom_xmat')
    _assert_attr_eq(d, dx, 'site_xpos')
    _assert_eq(d.site_xmat.reshape((-1, 3, 3)), dx.site_xmat, 'site_xmat')
    # com_pos
    dx = jax.jit(mjx.com_pos)(mx, mjx.put_data(m, d))
    _assert_attr_eq(d, dx, 'subtree_com')
    _assert_attr_eq(d, dx, 'cinert')
    _assert_attr_eq(d, dx, 'cdof')
    # camlight
    dx = jax.jit(mjx.camlight)(mx, mjx.put_data(m, d))
    _assert_attr_eq(d, dx, 'cam_xpos')
    _assert_eq(d.cam_xmat.reshape((-1, 3, 3)), dx.cam_xmat, 'cam_xmat')
    # crb
    dx = jax.jit(mjx.crb)(mx, mjx.put_data(m, d))
    _assert_attr_eq(d, dx, 'crb')
    _assert_attr_eq(d, dx, 'qM')
    # factor_m
    dx = jax.jit(mjx.factor_m)(mx, mjx.put_data(m, d))
    _assert_attr_eq(d, dx, 'qLD')
    _assert_attr_eq(d, dx, 'qLDiagInv')
    # com_vel
    dx = jax.jit(mjx.com_vel)(mx, mjx.put_data(m, d))
    _assert_attr_eq(d, dx, 'cvel')
    _assert_attr_eq(d, dx, 'cdof_dot')
    # rne
    dx = jax.jit(mjx.rne)(mx, mjx.put_data(m, d))
    _assert_attr_eq(d, dx, 'qfrc_bias')

    # set dense jacobian for tendon:
    m.opt.jacobian = mujoco.mjtJacobian.mjJAC_DENSE
    d = mujoco.MjData(m)
    # give the system a little kick to ensure we have non-identity rotations
    d.qvel = np.random.random(m.nv)
    mujoco.mj_step(m, d, 10)  # let dynamics get state significantly non-zero
    mujoco.mj_forward(m, d)
    # tendon
    dx = jax.jit(mjx.tendon)(mx, mjx.put_data(m, d))
    _assert_attr_eq(d, dx, 'ten_J')
    _assert_attr_eq(d, dx, 'ten_length')
    # transmission
    dx = jax.jit(mjx.transmission)(mx, dx)
    _assert_attr_eq(d, dx, 'actuator_length')
    _assert_attr_eq(d, dx, 'actuator_moment')

  def test_disable_gravity(self):
    m = mujoco.MjModel.from_xml_string("""
        <mujoco>
          <option>
            <flag gravity="disable"/>
          </option>
          <worldbody>
            <body>
              <joint type="free"/>
              <geom size="0.1"/>
            </body>
          </worldbody>
        </mujoco>
        """)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    mx = mjx.put_model(m)
    dx = mjx.put_data(m, d)

    dx = jax.jit(mjx.rne)(mx, dx)
    np.testing.assert_allclose(dx.qfrc_bias, 0)

  def test_site_transmission(self):
    m = mujoco.MjModel.from_xml_string("""
        <mujoco>
        <compiler autolimits="true"/>
        <worldbody>
          <body>
            <joint type="free"/>
            <geom type="box" size=".05 .05 .05" mass="1"/>
            <site name="site1"/>
            <body>
              <joint type="hinge"/>
              <geom size="0.1" mass="1"/>
              <site name="site2" pos="0.1 0.2 0.3"/>
            </body>
          </body>
          <body pos="1 0 0">
            <joint name="slide" type="hinge"/>
            <geom type="box" size=".05 .05 .05" mass="1"/>
          </body>
        </worldbody>
        <actuator>
          <position site="site1" kv="0.1" gear="1 2 3 0 0 0"/>
          <position site="site1" kv="0.2" gear="0 0 0 1 2 3"/>
          <position site="site2" kv="0.3" gear="0 3 0 0 0 1"/>
          <position joint="slide" kv="0.05" />
          <position site="site2" refsite="site1" gear="1 2 3 0.5 0.4 0.6"/>
        </actuator>
        </mujoco>
      """)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    mx = mjx.put_model(m)
    dx = mjx.put_data(m, d)

    mujoco.mj_transmission(m, d)
    dx = jax.jit(mjx.transmission)(mx, dx)
    _assert_attr_eq(d, dx, 'actuator_length')
    _assert_attr_eq(d, dx, 'actuator_moment')

  def test_subtree_vel(self):
    """Tests MJX subtree_vel function matches MuJoCo mj_subtreeVel."""

    m = test_util.load_test_file('humanoid/humanoid.xml')
    d = mujoco.MjData(m)
    # give the system a little kick to ensure we have non-identity rotations
    d.qvel = np.random.random(m.nv)
    mujoco.mj_step(m, d, 10)  # let dynamics get state significantly non-zero
    mujoco.mj_forward(m, d)
    mx = mjx.put_model(m)
    dx = mjx.put_data(m, d)

    # subtree velocity
    mujoco.mj_subtreeVel(m, d)
    dx = jax.jit(mjx.subtree_vel)(mx, dx)

    _assert_attr_eq(d, dx, 'subtree_linvel')
    _assert_attr_eq(d, dx, 'subtree_angmom')

  def test_rnepostconstraint(self):
    """Tests MJX rne_postconstraint function to match MuJoCo mj_rnePostConstraint."""

    m = mujoco.MjModel.from_xml_string("""
        <mujoco>
          <worldbody>
            <geom name="floor" size="0 0 .05" type="plane"/>
            <body pos="0 0 1">
              <joint type="ball" damping="1"/>
              <geom type="capsule" size="0.1 0.5" fromto="0 0 0 0.5 0 0"/>
              <body pos="0.5 0 0">
                <joint type="ball" damping="1"/>
                <geom type="capsule" size="0.1 0.5" fromto="0 0 0 0.5 0 0"/>
              </body>
            </body>
          </worldbody>
          <keyframe>
            <key qpos='0.424577 0.450592 0.451703 -0.642391 0.729379 0.545151 0.407756 0.0674697'/>
          </keyframe>
        </mujoco>
    """)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    # apply external forces
    d.xfrc_applied = 0.001 * np.ones(d.xfrc_applied.shape)
    mujoco.mj_step(m, d, 2)
    mujoco.mj_forward(m, d)
    mx = mjx.put_model(m)
    dx = mjx.put_data(m, d)

    # rne postconstraint
    mujoco.mj_rnePostConstraint(m, d)
    dx = jax.jit(mjx.rne_postconstraint)(mx, dx)

    _assert_eq(d.cacc, dx.cacc, 'cacc')
    _assert_eq(d.cfrc_ext, dx.cfrc_ext, 'cfrc_ext')
    _assert_eq(d.cfrc_int, dx.cfrc_int, 'cfrc_int')


class TendonTest(parameterized.TestCase):

  @parameterized.parameters(
      'tendon/fixed.xml',
      'tendon/site.xml',
      'tendon/fixed_site.xml',
      'tendon/no_tendon.xml',
  )
  def test_tendon(self, filename):
    """Tests MJX tendon function matches MuJoCo mj_tendon."""
    m = test_util.load_test_file(filename)
    d = mujoco.MjData(m)
    # give the system a little kick to ensure we have non-identity rotations
    d.qvel = np.random.random(m.nv)
    mujoco.mj_step(m, d, 10)  # let dynamics get state significantly non-zero
    mx = mjx.put_model(m)
    dx = mjx.put_data(m, d)

    mujoco.mj_forward(m, d)
    dx = jax.jit(mjx.forward)(mx, dx)

    _assert_eq(d.ten_length, dx.ten_length, 'ten_length')
    _assert_eq(d.ten_J, dx.ten_J, 'ten_J')
    _assert_eq(d.ten_wrapnum, dx.ten_wrapnum, 'ten_wrapnum')
    _assert_eq(d.ten_wrapadr, dx.ten_wrapadr, 'ten_wrapadr')
    if d.wrap_obj.shape == dx.wrap_obj.shape:
      _assert_eq(d.wrap_obj, dx.wrap_obj, 'wrap_obj')
    if d.wrap_xpos.shape == dx.wrap_xpos.shape:
      _assert_eq(d.wrap_xpos, dx.wrap_xpos, 'wrap_xpos')


if __name__ == '__main__':
  absltest.main()
