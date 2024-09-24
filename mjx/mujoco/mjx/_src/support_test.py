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


if __name__ == '__main__':
  absltest.main()
