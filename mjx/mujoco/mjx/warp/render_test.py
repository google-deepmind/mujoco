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

  def _maybe_skip(self):
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
    self._maybe_skip()
    mx, dx_batch, rc = _get_model_data_rc(xml, batch_size)

    dx_batch = jax.jit(mjx.refit_bvh)(mx, dx_batch, rc.pytree())
    out_batch = jax.jit(mjx.render)(mx, dx_batch, rc.pytree())

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
    self._maybe_skip()
    mx, dx_batch, rc = _get_model_data_rc(xml, batch_size)

    def inner(mx, dx, rc):
      dx = jax.vmap(bvh.refit_bvh, in_axes=(None, 0, None))(mx, dx, rc)
      out = jax.vmap(render.render, in_axes=(None, 0, None))(mx, dx, rc)
      return out

    # get reference with single vmap
    dx_batch = jax.vmap(bvh.refit_bvh, in_axes=(None, 0, None))(
        mx, dx_batch, rc.pytree()
    )
    ref = jax.vmap(render.render, in_axes=(None, 0, None))(
        mx, dx_batch, rc.pytree()
    )
    ref_rgb = np.asarray(ref[0])
    ref_depth = np.asarray(ref[1])

    # call with nested vmap
    def _reshape_batched(x):
      if x.shape[0] == batch_size:
        return x.reshape(2, batch_size // 2, *x.shape[1:])
      return x

    dx_2d = jax.tree.map(_reshape_batched, dx_batch)

    out_batch = jax.vmap(inner, in_axes=(None, 0, None))(mx, dx_2d, rc.pytree())
    out_batch = jax.tree.map(lambda x: x.reshape(-1, *x.shape[2:]), out_batch)
    rgb = np.asarray(out_batch[0])
    depth = np.asarray(out_batch[1])

    np.testing.assert_array_equal(rgb, ref_rgb)
    np.testing.assert_array_equal(depth, ref_depth)
    self.assertGreater(np.count_nonzero(rgb), 0)
    self.assertNotEqual(np.unique(rgb).shape[0], 1)
    self.assertGreater(np.count_nonzero(depth), 0)
    self.assertNotEqual(np.unique(depth).shape[0], 1)


class RenderContextGarbageCollectionTest(absltest.TestCase):
  """Tests that RenderContext cleans up buffers on deletion."""

  def setUp(self):
    super().setUp()
    if mjxw.WARP_INSTALLED:
      tempdir = '/tmp/wp_kernel_cache_dir_RenderContextGCTest'
      wp.config.kernel_cache_dir = tempdir

  def _maybe_skip(self):
    if not mjxw.WARP_INSTALLED:
      self.skipTest('Warp not installed.')
    if not io.has_cuda_gpu_device():
      self.skipTest('No CUDA GPU device available.')

  def test_render_context_gc(self):
    """Verifies __del__ removes entries from _MJX_RENDER_CONTEXT_BUFFERS."""
    self._maybe_skip()
    from mujoco.mjx.warp import render_context as rc_module  # pylint: disable=g-import-not-at-top

    self.assertEmpty(rc_module._MJX_RENDER_CONTEXT_BUFFERS)

    _, _, rc = _get_model_data_rc('humanoid/humanoid.xml', 1)
    key = rc.key

    # Sanity check: buffers exist for this key.
    matching = [
        k
        for k in rc_module._MJX_RENDER_CONTEXT_BUFFERS
        if isinstance(k, tuple) and k[0] == key
    ]
    self.assertNotEmpty(matching)

    # Delete the RenderContext and verify cleanup.
    del rc

    matching = [
        k
        for k in rc_module._MJX_RENDER_CONTEXT_BUFFERS
        if isinstance(k, tuple) and k[0] == key
    ]
    self.assertEmpty(matching)

  def test_render_context_gc_multi_keys(self):
    """Verifies deleting one context doesn't remove another's buffers."""
    self._maybe_skip()
    from mujoco.mjx.warp import render_context as rc_module  # pylint: disable=g-import-not-at-top

    self.assertEmpty(rc_module._MJX_RENDER_CONTEXT_BUFFERS)

    _, _, rc_a = _get_model_data_rc('humanoid/humanoid.xml', 1)
    _, _, rc_b = _get_model_data_rc('humanoid/humanoid.xml', 1)
    key_a = rc_a.key
    key_b = rc_b.key

    # rc_a's buffers should be present.
    matching_a = [
        k
        for k in rc_module._MJX_RENDER_CONTEXT_BUFFERS
        if isinstance(k, tuple) and k[0] == key_a
    ]
    self.assertNotEmpty(matching_a)

    # Delete only rc_a.
    del rc_a

    # rc_a's buffers should be gone.
    matching_a = [
        k
        for k in rc_module._MJX_RENDER_CONTEXT_BUFFERS
        if isinstance(k, tuple) and k[0] == key_a
    ]
    self.assertEmpty(matching_a)

    # rc_b's buffers should still be present.
    matching_b = [
        k
        for k in rc_module._MJX_RENDER_CONTEXT_BUFFERS
        if isinstance(k, tuple) and k[0] == key_b
    ]
    self.assertNotEmpty(matching_b)

    # Clean up rc_b.
    del rc_b
    matching_b = [
        k
        for k in rc_module._MJX_RENDER_CONTEXT_BUFFERS
        if isinstance(k, tuple) and k[0] == key_b
    ]
    self.assertEmpty(matching_b)


if __name__ == '__main__':
  absltest.main()
