# Copyright 2025 DeepMind Technologies Limited
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
"""Tests for collision driver."""
import os
import tempfile

from absl.testing import absltest
import jax
import mujoco
from mujoco import mjx
from mujoco.mjx._src import io
import mujoco.mjx.warp as mjxw
from mujoco.mjx.warp import test_util as tu
from mujoco.mjx.warp import warp as wp  # pylint: disable=g-importing-member
import numpy as np


try:
  from mujoco.mjx.warp import collision_driver  # pylint: disable=g-import-not-at-top
  from mujoco.mjx.warp import smooth  # pylint: disable=g-import-not-at-top
except ImportError:
  collision_driver = None
  smooth = None

_FORCE_TEST = os.environ.get('MJX_WARP_FORCE_TEST', '0') == '1'


class CollisionTest(absltest.TestCase):

  def setUp(self):
    super().setUp()
    if mjxw.WARP_INSTALLED:
      self.tempdir = tempfile.TemporaryDirectory()
      wp.config.kernel_cache_dir = self.tempdir.name
    np.random.seed(0)

  def tearDown(self):
    super().tearDown()
    if hasattr(self, 'tempdir'):
      self.tempdir.cleanup()

  _SPHERE_SPHERE = """
    <mujoco>
      <worldbody>
        <body>
          <joint type="free"/>
          <geom pos="0 0 0" size="0.2" type="sphere"/>
        </body>
        <body >
          <joint type="free"/>
          <geom pos="0 0.3 0" size="0.11" type="sphere"/>
        </body>
      </worldbody>
    </mujoco>
  """

  def test_collision_nested_vmap(self):
    """Tests collision with batched data."""
    if not _FORCE_TEST:
      if not mjxw.WARP_INSTALLED:
        self.skipTest('Warp not installed.')
      if not io.has_cuda_gpu_device():
        self.skipTest('No CUDA GPU device available.')

    m = mujoco.MjModel.from_xml_string(self._SPHERE_SPHERE)
    d = mujoco.MjData(m)
    mx = mjx.put_model(m, impl='warp')

    def make_data(rng):
      dx = mjx.make_data(m, impl='warp')
      _, key = jax.random.split(rng)
      qpos = jax.random.uniform(key, (m.nq,), minval=-0.01, maxval=0.01)
      return dx.replace(
          qpos=qpos,
      )

    rng = jax.random.split(jax.random.PRNGKey(0), 8)
    rng = rng.reshape((2, 4, -1))
    dx_batch = jax.vmap(jax.vmap(make_data))(rng)

    dx_batch = jax.jit(
        jax.vmap(
            jax.vmap(smooth.kinematics, in_axes=(None, 0)), in_axes=(None, 0)
        )
    )(mx, dx_batch)
    dx_batch = jax.jit(
        jax.vmap(
            jax.vmap(collision_driver.collision, in_axes=(None, 0)),
            in_axes=(None, 0),
        )
    )(mx, dx_batch)

    for i in range(2):
      for j in range(4):
        dx = dx_batch[i, j]

        d.qpos[:] = dx.qpos
        mujoco.mj_forward(m, d)

        if not d.contact.pos.shape[0]:
          continue
        tu.assert_contact_eq(d, dx, worldid=i * 4 + j)


if __name__ == '__main__':
  absltest.main()
