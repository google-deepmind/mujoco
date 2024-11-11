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
"""Tests for support."""

from absl.testing import absltest
from absl.testing import parameterized
import jax
from jax import numpy as jp
import mujoco
from mujoco import mjx
from mujoco.mjx._src import support
from mujoco.mjx._src import test_util
import numpy as np


class SupportTest(parameterized.TestCase):

  def test_mul_m(self):
    m = test_util.load_test_file('pendula.xml')
    # first test sparse
    m.opt.jacobian = mujoco.mjtJacobian.mjJAC_SPARSE
    d = mujoco.MjData(m)
    # give the system a little kick to ensure we have non-identity rotations
    d.qvel = np.random.random(m.nv)
    mujoco.mj_step(m, d, 10)  # let dynamics get state significantly non-zero
    mujoco.mj_forward(m, d)
    mx = mjx.put_model(m)
    dx = mjx.put_data(m, d)
    vec = np.random.random(m.nv)
    mjx_vec = jax.jit(mjx.mul_m)(mx, dx, jp.array(vec))
    mj_vec = np.zeros(m.nv)
    mujoco.mj_mulM(m, d, mj_vec, vec)
    np.testing.assert_allclose(mjx_vec, mj_vec, atol=5e-5, rtol=5e-5)

    # also check dense
    m.opt.jacobian = mujoco.mjtJacobian.mjJAC_DENSE
    mujoco.mj_forward(m, d)
    mx = mjx.put_model(m)
    dx = mjx.put_data(m, d)
    mjx_vec = jax.jit(mjx.mul_m)(mx, dx, jp.array(vec))
    np.testing.assert_allclose(mjx_vec, mj_vec, atol=5e-5, rtol=5e-5)

  def test_full_m(self):
    m = test_util.load_test_file('pendula.xml')
    # for the model to be sparse to exercise MJX full_M
    m.opt.jacobian = mujoco.mjtJacobian.mjJAC_SPARSE
    d = mujoco.MjData(m)
    # give the system a little kick to ensure we have non-identity rotations
    d.qvel = np.random.random(m.nv)
    mujoco.mj_step(m, d, 10)  # let dynamics get state significantly non-zero
    mx = mjx.put_model(m)
    dx = mjx.put_data(m, d)
    mjx_full_m = jax.jit(support.full_m)(mx, dx)
    mj_full_m = np.zeros((m.nv, m.nv), dtype=np.float64)
    mujoco.mj_fullM(m, mj_full_m, d.qM)
    np.testing.assert_allclose(mjx_full_m, mj_full_m, atol=5e-5, rtol=5e-5)

  @parameterized.parameters('constraints.xml', 'pendula.xml')
  def test_jac(self, fname):
    np.random.seed(0)

    m = test_util.load_test_file(fname)
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d)
    mx = mjx.put_model(m)
    dx = mjx.put_data(m, d)
    point = np.random.randn(3)
    body = np.random.choice(m.nbody)
    jacp, jacr = jax.jit(support.jac)(mx, dx, point, body)

    jacp_expected, jacr_expected = np.zeros((3, m.nv)), np.zeros((3, m.nv))
    mujoco.mj_jac(m, d, jacp_expected, jacr_expected, point, body)
    np.testing.assert_almost_equal(jacp, jacp_expected.T, 6)
    np.testing.assert_almost_equal(jacr, jacr_expected.T, 6)

  def test_xfrc_accumulate(self):
    """Tests that xfrc_accumulate ouput matches mj_xfrcAccumulate."""
    np.random.seed(0)

    m = test_util.load_test_file('pendula.xml')
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d)
    mx = mjx.put_model(m)
    dx = mjx.put_data(m, d)
    self.assertFalse((dx.xipos == 0.0).all())

    xfrc = np.random.rand(*dx.xfrc_applied.shape)

    d.xfrc_applied[:] = xfrc
    dx = dx.replace(xfrc_applied=jp.array(xfrc))

    qfrc = jax.jit(support.xfrc_accumulate)(mx, dx)
    qfrc_expected = np.zeros(m.nv)
    for i in range(1, m.nbody):
      mujoco.mj_applyFT(
          m,
          d,
          d.xfrc_applied[i, :3],
          d.xfrc_applied[i, 3:],
          d.xipos[i],
          i,
          qfrc_expected,
      )

    np.testing.assert_almost_equal(qfrc, qfrc_expected, 6)

  def test_custom(self):
    xml = """
    <mujoco model="right_shadow_hand">
        <custom>
          <numeric data="15" name="max_contact_points"/>
          <numeric data="42" name="max_geom_pairs"/>
        </custom>
    </mujoco>
    """
    m = mujoco.MjModel.from_xml_string(xml)

    def _get_numeric(m, name):
      id_ = support.name2id(m, mujoco.mjtObj.mjOBJ_NUMERIC, name)
      return int(m.numeric_data[id_]) if id_ >= 0 else -1

    self.assertEqual(_get_numeric(m, 'something'), -1)
    self.assertEqual(_get_numeric(m, 'max_contact_points'), 15)
    self.assertEqual(_get_numeric(m, 'max_geom_pairs'), 42)

    mx = mjx.put_model(m)
    self.assertEqual(_get_numeric(mx, 'something'), -1)
    self.assertEqual(_get_numeric(mx, 'max_contact_points'), 15)
    self.assertEqual(_get_numeric(mx, 'max_geom_pairs'), 42)

  def test_names_and_ids(self):
    m = test_util.load_test_file('pendula.xml')
    mx = mjx.put_model(m)

    nums = {
        mujoco.mjtObj.mjOBJ_JOINT: m.njnt,
        mujoco.mjtObj.mjOBJ_GEOM: m.ngeom,
        mujoco.mjtObj.mjOBJ_BODY: m.nbody,
    }

    for obj in nums:
      names = [mujoco.mj_id2name(m, obj.value, i) for i in range(nums[obj])]
      for i, n in enumerate(names):
        self.assertEqual(support.id2name(mx, obj, i), n)
        i = i if n is not None else -1
        self.assertEqual(support.name2id(mx, obj, n), i)

  _CONTACTS = """
    <mujoco>
      <worldbody>
        <body pos="0 0 0.55" euler="1 0 0">
          <joint axis="1 0 0" type="free"/>
          <geom fromto="-0.4 0 0 0.4 0 0" size="0.05" type="capsule" condim="6"/>
        </body>
        <body pos="0 0 0.5" euler="0 1 0">
          <joint axis="1 0 0" type="free"/>
          <geom fromto="-0.4 0 0 0.4 0 0" size="0.05" type="capsule" condim="3"/>
        </body>
        <body pos="0 0 0.445" euler="0 90 0">
          <joint axis="1 0 0" type="free"/>
          <geom fromto="-0.4 0 0 0.4 0 0" size="0.05" type="capsule" condim="1"/>
        </body>
      </worldbody>
    </mujoco>
  """

  def test_contact_force(self):
    m = mujoco.MjModel.from_xml_string(self._CONTACTS)
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d)
    assert (
        np.unique(d.contact.geom).shape[0] == 3
    ), 'This test assumes all capsule are in contact.'
    mx = mjx.put_model(m)
    dx = mjx.put_data(m, d)
    mujoco.mj_step(m, d)
    dx = mjx.step(mx, dx)

    # map MJX contacts to MJ ones
    def _find(g):
      val = (g == dx.contact.geom).sum(axis=1)
      return np.where(val == 2)[0][0]

    contact_id_map = {i: _find(d.contact.geom[i]) for i in range(d.ncon)}

    for i in range(d.ncon):
      result = np.zeros(6, dtype=float)
      mujoco.mj_contactForce(m, d, i, result)

      j = contact_id_map[i]
      force = jax.jit(support.contact_force, static_argnums=(2,))(mx, dx, j)
      np.testing.assert_allclose(result, force, rtol=1e-5, atol=2)

      # check for zeros after first condim elements
      condim = dx.contact.dim[j]
      if condim < 6:
        np.testing.assert_allclose(force[condim:], 0, rtol=1e-5, atol=1e-5)

      # test world conversion
      force = jax.jit(
          support.contact_force,
          static_argnums=(
              2,
              3,
          ),
      )(mx, dx, j, True)
      # back to contact frame
      force = force.at[:3].set(dx.contact.frame[j] @ force[:3])
      force = force.at[3:].set(dx.contact.frame[j] @ force[3:])
      np.testing.assert_allclose(result, force, rtol=1e-5, atol=2)

  def test_muscle_gain_length(self):
    lmin = 0.5
    lmax = 1.5
    np.testing.assert_allclose(
        support.muscle_gain_length(0, lmin, lmax),
        jp.zeros(1),
        rtol=1e-5,
        atol=1e-5,
    )
    np.testing.assert_allclose(
        support.muscle_gain_length(0.5, lmin, lmax),
        jp.zeros(1),
        rtol=1e-5,
        atol=1e-5,
    )
    np.testing.assert_allclose(
        support.muscle_gain_length(0.6, lmin, lmax),
        jp.array([0.08]),
        rtol=1e-5,
        atol=1e-5,
    )
    np.testing.assert_allclose(
        support.muscle_gain_length(0.75, lmin, lmax),
        jp.array([0.5]),
        rtol=1e-5,
        atol=1e-5,
    )
    np.testing.assert_allclose(
        support.muscle_gain_length(1.0, lmin, lmax),
        jp.ones(1),
        rtol=1e-5,
        atol=1e-5,
    )
    np.testing.assert_allclose(
        support.muscle_gain_length(1.25, lmin, lmax),
        jp.array([0.5]),
        rtol=1e-5,
        atol=1e-5,
    )
    np.testing.assert_allclose(
        support.muscle_gain_length(1.5, lmin, lmax),
        jp.zeros(1),
        rtol=1e-5,
        atol=1e-5,
    )
    np.testing.assert_allclose(
        support.muscle_gain_length(2.0, lmin, lmax),
        jp.zeros(1),
        rtol=1e-5,
        atol=1e-5,
    )

  def test_muscle_gain(self):
    length = jp.array([1.0])
    lengthrange = jp.array([0.0, 1.0])
    acc0 = jp.array([1.0])
    prm = jp.array([0.0, 1.0, 1.0, 200.0, 0.5, 3.0, 1.0, 0.0, 2.0, 0.0])

    # V <= -1
    vel = jp.array([-1.5])
    np.testing.assert_allclose(
        support.muscle_gain(length, vel, lengthrange, acc0, prm),
        jp.array([-0.0]),
        rtol=1e-5,
        atol=1e-5,
    )

    # V <= 0
    vel = jp.array([-0.5])
    np.testing.assert_allclose(
        support.muscle_gain(length, vel, lengthrange, acc0, prm),
        jp.array([-0.25]),
        rtol=1e-5,
        atol=1e-5,
    )

    # V <= y
    vel = jp.array([0.5])
    np.testing.assert_allclose(
        support.muscle_gain(length, vel, lengthrange, acc0, prm),
        jp.array([-1.75]),
        rtol=1e-5,
        atol=1e-5,
    )

    # V > y
    vel = jp.array([1.5])
    np.testing.assert_allclose(
        support.muscle_gain(length, vel, lengthrange, acc0, prm),
        jp.array([-2.0]),
        rtol=1e-5,
        atol=1e-5,
    )

    # force < 0
    prm = prm.at[2].set(-1.0)
    np.testing.assert_allclose(
        support.muscle_gain(length, vel, lengthrange, acc0, prm),
        jp.array([-400.0]),
        rtol=1e-5,
        atol=1e-5,
    )

  def test_muscle_bias(self):
    lengthrange = jp.array([0.0, 1.0])
    acc0 = jp.array([1.0])
    prm = jp.array([0.0, 1.0, 1.0, 200.0, 0.5, 3.0, 1.5, 1.3, 1.2, 0.0])

    # L <= 1
    length = jp.array([0.5])
    np.testing.assert_allclose(
        support.muscle_bias(length, lengthrange, acc0, prm),
        jp.array([0.0]),
        rtol=1e-5,
        atol=1e-5,
    )

    # L <= b
    length = jp.array([1.5])
    np.testing.assert_allclose(
        support.muscle_bias(length, lengthrange, acc0, prm),
        jp.array([-0.1625]),
        rtol=1e-5,
        atol=1e-5,
    )

    # L > b
    length = jp.array([2.5])
    np.testing.assert_allclose(
        support.muscle_bias(length, lengthrange, acc0, prm),
        jp.array([-1.3]),
        rtol=1e-5,
        atol=1e-5,
    )

    # force < 0
    prm = prm.at[2].set(-1.0)
    np.testing.assert_allclose(
        support.muscle_bias(length, lengthrange, acc0, prm),
        jp.array([-260.0]),
        rtol=1e-5,
        atol=1e-5,
    )

  def test_smooth_muscle_dynamics(self):
    # compute time constant as in Millard et al. (2013)
    # https://doi.org/10.1115/1.4023390
    def _muscle_dynamics_millard(ctrl, act, prm):
      ctrlclamp = jp.clip(ctrl, 0, 1)
      actclamp = jp.clip(act, 0, 1)

      tau0 = prm[0] * (0.5 + 1.5 * actclamp)
      tau1 = prm[1] / (0.5 + 1.5 * actclamp)
      tau = jp.where(ctrlclamp > act, tau0, tau1)

      return (ctrlclamp - act) / jp.maximum(mujoco.mjMINVAL, tau)

    prm = jp.array([0.01, 0.04, 0.0])

    # exact equality if tau_smooth = 0
    for ctrl in [-0.1, 0.0, 0.4, 0.5, 1.0, 1.0]:
      for act in [-0.1, 0.0, 0.4, 0.5, 1.0, 1.1]:
        actdot_old = _muscle_dynamics_millard(ctrl, act, prm)
        actdot_new = support.muscle_dynamics(ctrl, act, prm)
        np.testing.assert_allclose(actdot_old, actdot_new, rtol=1e-5, atol=1e-5)

    # positive tau_smooth
    tau_smooth = 0.2
    prm = prm.at[2].set(tau_smooth)
    act = 0.5
    eps = 1.0e-6

    ctrl = 0.4 - eps  # smaller than act by just over 0.5 * tau_smooth
    np.testing.assert_allclose(
        _muscle_dynamics_millard(ctrl, act, prm),
        support.muscle_dynamics(ctrl, act, prm),
        rtol=1e-5,
        atol=1e-5,
    )

    ctrl = 0.6 + eps  # larger than act by just over 0.5 * tau_smooth
    np.testing.assert_allclose(
        _muscle_dynamics_millard(ctrl, act, prm),
        support.muscle_dynamics(ctrl, act, prm),
        rtol=1e-5,
        atol=1e-5,
    )

    # right in the middle should give average of time constants
    tau_act = 0.2
    tau_deact = 0.3
    for dctrl in [0.0, 0.1, 0.2, 1.0, 1.1]:
      lower = support.muscle_dynamics_timescale(
          -dctrl, tau_act, tau_deact, tau_smooth
      )
      upper = support.muscle_dynamics_timescale(
          dctrl, tau_act, tau_deact, tau_smooth
      )
      np.testing.assert_allclose(
          0.5 * (upper + lower),
          0.5 * (tau_act + tau_deact),
          rtol=1e-5,
          atol=1e-5,
      )


if __name__ == '__main__':
  absltest.main()
