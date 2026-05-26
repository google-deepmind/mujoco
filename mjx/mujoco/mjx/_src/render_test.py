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
"""Integration tests for render + get_rgb / get_depth / get_segmentation."""

import functools
import os

from absl.testing import absltest
from absl.testing import parameterized
import jax
from jax import numpy as jp
import mujoco
from mujoco import mjx
from mujoco.mjx._src import forward
from mujoco.mjx._src import io
import mujoco.mjx.warp as mjxw
from mujoco.mjx.warp import test_util as tu
import numpy as np

_FORCE_TEST = os.environ.get('MJX_WARP_FORCE_TEST', '0') == '1'

_WIDTH, _HEIGHT = 32, 32


def _setup(batch_size):
  """Returns (mx, dx_batch, rc) for humanoid with segmentation enabled."""
  m = tu.load_test_file('humanoid/humanoid.xml')
  d = mujoco.MjData(m)
  mujoco.mj_forward(m, d)

  mx = mjx.put_model(m, impl='warp')
  worldids = jp.arange(batch_size)
  dx_batch = jax.vmap(functools.partial(tu.make_data, m))(worldids)
  dx_batch = jax.jit(jax.vmap(forward.forward, in_axes=(None, 0)))(
      mx, dx_batch
  )

  rc = mjx.create_render_context(
      mjm=m,
      nworld=batch_size,
      cam_res=(_WIDTH, _HEIGHT),
      render_rgb=True,
      render_depth=True,
      render_seg=True,
      enabled_geom_groups=[0, 1, 2],
  )
  dx_batch = jax.jit(mjx.refit_bvh)(mx, dx_batch, rc.pytree())
  return mx, dx_batch, rc


class RenderIntegrationTest(parameterized.TestCase):
  """Tests the full render → unpack pipeline."""

  def setUp(self):
    super().setUp()
    if mjxw.WARP_INSTALLED:
      import warp  # pylint: disable=g-import-not-at-top
      warp.config.kernel_cache_dir = '/tmp/wp_kernel_cache_dir_RenderIntTest'
    np.random.seed(0)

  def _maybe_skip(self):
    if not _FORCE_TEST:
      if not mjxw.WARP_INSTALLED:
        self.skipTest('Warp not installed.')
      if not io.has_cuda_gpu_device():
        self.skipTest('No CUDA GPU device available.')

  @parameterized.parameters(1, 4)
  def test_render_unpack(self, batch_size):
    """render_with_segmentation → get_rgb / get_depth / get_segmentation."""
    self._maybe_skip()
    mx, dx_batch, rc = _setup(batch_size)

    rgb_packed, depth_packed, seg_packed = jax.jit(
        mjx.render_with_segmentation
    )(mx, dx_batch, rc.pytree())

    rc_pytree = rc.pytree()
    rgb = mjx.get_rgb(rc_pytree, 0, rgb_packed)
    depth = mjx.get_depth(rc_pytree, 0, depth_packed, 5.0)
    seg = mjx.get_segmentation(rc_pytree, 0, seg_packed)

    self.assertEqual(rgb.shape, (batch_size, _HEIGHT, _WIDTH, 3))
    self.assertEqual(depth.shape, (batch_size, _HEIGHT, _WIDTH, 1))
    self.assertEqual(seg.shape, (batch_size, _HEIGHT, _WIDTH))
    self.assertGreater(np.count_nonzero(np.asarray(rgb)), 0)
    self.assertGreater(np.count_nonzero(np.asarray(depth)), 0)
    self.assertTrue(np.any(np.asarray(seg) != -1))

  @parameterized.parameters((4,),)
  def test_render_unpack_vmap(self, batch_size):
    """render_with_segmentation → vmap(get_rgb / get_depth / get_seg)."""
    self._maybe_skip()
    mx, dx_batch, rc = _setup(batch_size)

    rgb_packed, depth_packed, seg_packed = jax.jit(
        mjx.render_with_segmentation
    )(mx, dx_batch, rc.pytree())

    rc_pytree = rc.pytree()
    rgb = jax.vmap(mjx.get_rgb, in_axes=(None, None, 0))(
        rc_pytree, 0, rgb_packed
    )
    depth = jax.vmap(mjx.get_depth, in_axes=(None, None, 0, None))(
        rc_pytree, 0, depth_packed, 5.0
    )
    seg = jax.vmap(mjx.get_segmentation, in_axes=(None, None, 0))(
        rc_pytree, 0, seg_packed
    )

    self.assertEqual(rgb.shape, (batch_size, _HEIGHT, _WIDTH, 3))
    self.assertEqual(depth.shape, (batch_size, _HEIGHT, _WIDTH, 1))
    self.assertEqual(seg.shape, (batch_size, _HEIGHT, _WIDTH))
    self.assertGreater(np.count_nonzero(np.asarray(rgb)), 0)
    self.assertGreater(np.count_nonzero(np.asarray(depth)), 0)
    self.assertTrue(np.any(np.asarray(seg) != -1))


if __name__ == '__main__':
  absltest.main()
