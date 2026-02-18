# Copyright 2026 DeepMind Technologies Limited
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
import functools
import os

from absl.testing import absltest
from absl.testing import parameterized
import jax
from jax import numpy as jp
import mujoco
from mujoco import mjx
from mujoco.mjx._src import bvh
from mujoco.mjx._src import forward
from mujoco.mjx._src import io
from mujoco.mjx._src import render
import mujoco.mjx.warp as mjxw
from mujoco.mjx.warp import test_util as tu
from mujoco.mjx.warp import warp as wp  # pylint: disable=g-importing-member
import numpy as np


_FORCE_TEST = os.environ.get('MJX_WARP_FORCE_TEST', '0') == '1'


def _get_model_data_rc(xml, batch_size):
  m = tu.load_test_file(xml)
  d = mujoco.MjData(m)
  mujoco.mj_forward(m, d)

  mx = mjx.put_model(m, impl='warp')

  worldids = jp.arange(batch_size)
  dx_batch = jax.vmap(functools.partial(tu.make_data, m))(worldids)

  key = jax.random.PRNGKey(0)
  keys = jax.random.split(key, batch_size)
  qpos0 = jp.array(m.qpos0)
  rand_qpos = jax.vmap(
      lambda k: qpos0 + jax.random.uniform(k, (m.nq,), minval=-0.2, maxval=0.05)
  )(keys)
  dx_batch = jax.vmap(lambda dx, q: dx.replace(qpos=q))(dx_batch, rand_qpos)

  dx_batch = jax.jit(jax.vmap(forward.forward, in_axes=(None, 0)))(mx, dx_batch)

  width, height = 32, 32
  rc = mjx.create_render_context(
      mjm=m,
      nworld=batch_size,
      cam_res=(width, height),
      use_textures=True,
      use_shadows=True,
      render_rgb=True,
      render_depth=True,
      enabled_geom_groups=[0, 1, 2],
  )
  return mx, dx_batch, rc


class RenderTest(parameterized.TestCase):

  def setUp(self):
    super().setUp()
    if mjxw.WARP_INSTALLED:
      tempdir = '/tmp/wp_kernel_cache_dir_RenderTest'
      wp.config.kernel_cache_dir = tempdir
    np.random.seed(0)

  def _skip_if_no_warp(self):
    if not _FORCE_TEST:
      if not mjxw.WARP_INSTALLED:
        self.skipTest('Warp not installed.')
      if not io.has_cuda_gpu_device():
        self.skipTest('No CUDA GPU device available.')

  @parameterized.product(
      xml=('humanoid/humanoid.xml',),
      batch_size=(1, 16),
  )
  def test_render(self, xml: str, batch_size: int):
    """Tests MJX render pipeline."""
    self._skip_if_no_warp()
    mx, dx_batch, rc = _get_model_data_rc(xml, batch_size)

    dx_batch = jax.jit(mjx.refit_bvh)(mx, dx_batch, rc)
    out_batch = jax.jit(mjx.render)(mx, dx_batch, rc)

    rgb = np.asarray(out_batch[0])
    depth = np.asarray(out_batch[1])

    self.assertGreater(np.count_nonzero(rgb), 0)
    self.assertGreater(np.count_nonzero(depth), 0)
    self.assertNotEqual(np.unique(rgb).shape[0], 1)
    self.assertNotEqual(np.unique(depth).shape[0], 1)

  @parameterized.product(
      xml=('humanoid/humanoid.xml',),
      batch_size=(4, 16),
  )
  def test_render_nested_vmap(self, xml: str, batch_size: int):
    """Tests MJX render pipeline with nested vmap."""
    self._skip_if_no_warp()
    mx, dx_batch, rc = _get_model_data_rc(xml, batch_size)

    def inner(mx, dx, rc):
      dx = jax.vmap(bvh.refit_bvh, in_axes=(None, 0, None))(mx, dx, rc)
      out = jax.vmap(render.render, in_axes=(None, 0, None))(mx, dx, rc)
      return out

    # get reference with single vmap
    dx_batch = jax.vmap(bvh.refit_bvh, in_axes=(None, 0, None))(
        mx, dx_batch, rc
    )
    ref = jax.vmap(render.render, in_axes=(None, 0, None))(mx, dx_batch, rc)
    ref_rgb = np.asarray(ref[0])
    ref_depth = np.asarray(ref[1])

    # call with nested vmap
    def _reshape_batched(x):
      if x.shape[0] == batch_size:
        return x.reshape(2, batch_size // 2, *x.shape[1:])
      return x

    dx_2d = jax.tree.map(_reshape_batched, dx_batch)

    out_batch = jax.vmap(inner, in_axes=(None, 0, None))(mx, dx_2d, rc)
    out_batch = jax.tree.map(lambda x: x.reshape(-1, *x.shape[2:]), out_batch)
    rgb = np.asarray(out_batch[0])
    depth = np.asarray(out_batch[1])

    np.testing.assert_array_equal(rgb, ref_rgb)
    np.testing.assert_array_equal(depth, ref_depth)
    self.assertGreater(np.count_nonzero(rgb), 0)
    self.assertNotEqual(np.unique(rgb).shape[0], 1)
    self.assertGreater(np.count_nonzero(depth), 0)
    self.assertNotEqual(np.unique(depth).shape[0], 1)


if __name__ == '__main__':
  absltest.main()
