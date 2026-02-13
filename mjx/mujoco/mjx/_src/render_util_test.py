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
import os
from unittest import mock

from absl.testing import absltest
import jax
import jax.numpy as jnp
from mujoco.mjx._src import io
from mujoco.mjx._src import render_util
import mujoco.mjx.warp as mjxw
import numpy as np

_FORCE_TEST = os.environ.get('MJX_WARP_FORCE_TEST', '0') == '1'


def _fake_render_context(ncam, width, height):
  """Fake RenderContext for testing."""
  rc = mock.MagicMock()
  rgb_adr = np.arange(ncam, dtype=np.int32) * width * height
  depth_adr = np.arange(ncam, dtype=np.int32) * width * height
  cam_res = np.tile([width, height], (ncam, 1)).astype(np.int32)
  rc.rgb_adr.numpy.return_value = rgb_adr
  rc.depth_adr.numpy.return_value = depth_adr
  rc.cam_res.numpy.return_value = cam_res
  return rc


class RenderUtilTest(absltest.TestCase):

  def setUp(self):
    super().setUp()
    if not _FORCE_TEST:
      if not mjxw.WARP_INSTALLED:
        self.skipTest('Warp not installed.')
      if not io.has_cuda_gpu_device():
        self.skipTest('No CUDA GPU device available.')

  def test_get_rgb(self):
    width, height = 4, 4
    warp_rc = _fake_render_context(1, width, height)
    rc = mock.MagicMock(key=0)
    rgb_data = jnp.zeros((width * height,), dtype=jnp.uint32)

    with mock.patch.dict(
        'mujoco.mjx.warp.render._MJX_RENDER_CONTEXT_BUFFERS',
        {0: warp_rc},
    ):
      rgb = jax.jit(render_util.get_rgb, static_argnums=(0, 2))(
          rc, rgb_data, 0
      )

    self.assertEqual(rgb.shape, (height, width, 3))

  def test_get_rgb_vmap(self):
    nworld, width, height = 3, 4, 4
    warp_rc = _fake_render_context(1, width, height)
    rc = mock.MagicMock(key=0)
    rgb_data = jnp.zeros(
        (nworld, width * height), dtype=jnp.uint32
    )

    with mock.patch.dict(
        'mujoco.mjx.warp.render._MJX_RENDER_CONTEXT_BUFFERS',
        {0: warp_rc},
    ):
      rgb = jax.jit(
          jax.vmap(render_util.get_rgb, in_axes=(None, 0, None)),
          static_argnums=(0, 2),
      )(rc, rgb_data, 0)

    self.assertEqual(rgb.shape, (nworld, height, width, 3))

  def test_get_depth(self):
    width, height = 4, 4
    warp_rc = _fake_render_context(1, width, height)
    rc = mock.MagicMock(key=0)
    depth_data = jnp.zeros((width * height,), dtype=jnp.float32)

    with mock.patch.dict(
        'mujoco.mjx.warp.render._MJX_RENDER_CONTEXT_BUFFERS',
        {0: warp_rc},
    ):
      depth = jax.jit(
          render_util.get_depth, static_argnums=(0, 2, 3)
      )(rc, depth_data, 0, 5.0)

    self.assertEqual(depth.shape, (height, width))

  def test_get_depth_vmap(self):
    nworld, width, height = 3, 4, 4
    warp_rc = _fake_render_context(1, width, height)
    rc = mock.MagicMock(key=0)
    depth_data = jnp.zeros(
        (nworld, width * height), dtype=jnp.float32
    )

    with mock.patch.dict(
        'mujoco.mjx.warp.render._MJX_RENDER_CONTEXT_BUFFERS',
        {0: warp_rc},
    ):
      depth = jax.jit(
          jax.vmap(render_util.get_depth, in_axes=(None, 0, None, None)),
          static_argnums=(0, 2, 3),
      )(rc, depth_data, 0, 5.0)

    self.assertEqual(depth.shape, (nworld, height, width))


if __name__ == '__main__':
  absltest.main()
