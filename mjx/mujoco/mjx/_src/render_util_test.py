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
import numpy as np

from mujoco.mjx._src import io
from mujoco.mjx._src import render_util
import mujoco.mjx.warp as mjxw
from mujoco.mjx.warp.render_context import RenderContextPytree

_FORCE_TEST = os.environ.get('MJX_WARP_FORCE_TEST', '0') == '1'


def _fake_render_context(ncam, width, height, render_seg=True):
  """Fake RenderContext for testing."""
  rc = mock.MagicMock()
  rgb_adr = np.arange(ncam, dtype=np.int32) * width * height
  depth_adr = np.arange(ncam, dtype=np.int32) * width * height
  if isinstance(render_seg, bool):
    render_seg = [render_seg] * ncam
  seg_adr = np.full(ncam, -1, dtype=np.int32)
  seg_offset = 0
  for i, enabled in enumerate(render_seg):
    if enabled:
      seg_adr[i] = seg_offset
      seg_offset += width * height
  cam_res = np.tile([width, height], (ncam, 1)).astype(np.int32)
  rc.rgb_adr.numpy.return_value = rgb_adr
  rc.depth_adr.numpy.return_value = depth_adr
  rc.seg_adr.numpy.return_value = seg_adr
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
    rc = mock.MagicMock(spec=RenderContextPytree, key=0)
    rgb_data = jnp.zeros((width * height,), dtype=jnp.uint32)

    with mock.patch.dict(
        'mujoco.mjx.warp.render_context._MJX_RENDER_CONTEXT_BUFFERS',
        {(0, None): warp_rc},
    ):
      rgb = jax.jit(render_util.get_rgb, static_argnums=(0, 1))(rc, 0, rgb_data)

    self.assertEqual(rgb.shape, (height, width, 3))

  def test_get_rgb_preserves_leading_dims(self):
    width, height = 4, 4
    warp_rc = _fake_render_context(1, width, height)
    rc = mock.MagicMock(spec=RenderContextPytree, key=0)

    with mock.patch.dict(
        'mujoco.mjx.warp.render_context._MJX_RENDER_CONTEXT_BUFFERS',
        {(0, None): warp_rc},
    ):
      for leading_shape in ((1,), (3,), (2, 3)):
        with self.subTest(leading_shape=leading_shape):
          rgb_data = jnp.zeros(
              leading_shape + (width * height,), dtype=jnp.uint32
          )
          rgb = jax.jit(render_util.get_rgb, static_argnums=(0, 1))(
              rc, 0, rgb_data
          )

          self.assertEqual(rgb.shape, leading_shape + (height, width, 3))

  def test_get_rgb_vmap(self):
    nworld, width, height = 3, 4, 4
    warp_rc = _fake_render_context(1, width, height)
    rc = mock.MagicMock(spec=RenderContextPytree, key=0)
    rgb_data = jnp.zeros((nworld, width * height), dtype=jnp.uint32)

    with mock.patch.dict(
        'mujoco.mjx.warp.render_context._MJX_RENDER_CONTEXT_BUFFERS',
        {(0, None): warp_rc},
    ):
      rgb = jax.jit(
          jax.vmap(render_util.get_rgb, in_axes=(None, None, 0)),
          static_argnums=(0, 1),
      )(rc, 0, rgb_data)

    self.assertEqual(rgb.shape, (nworld, height, width, 3))

  def test_get_depth(self):
    width, height = 4, 4
    warp_rc = _fake_render_context(1, width, height)
    rc = mock.MagicMock(spec=RenderContextPytree, key=0)
    depth_data = jnp.zeros((width * height,), dtype=jnp.float32)

    with mock.patch.dict(
        'mujoco.mjx.warp.render_context._MJX_RENDER_CONTEXT_BUFFERS',
        {(0, None): warp_rc},
    ):
      depth = jax.jit(render_util.get_depth, static_argnums=(0, 1, 3))(
          rc, 0, depth_data, 5.0
      )

    self.assertEqual(depth.shape, (height, width, 1))

  def test_get_depth_preserves_leading_dims(self):
    width, height = 4, 4
    warp_rc = _fake_render_context(1, width, height)
    rc = mock.MagicMock(spec=RenderContextPytree, key=0)

    with mock.patch.dict(
        'mujoco.mjx.warp.render_context._MJX_RENDER_CONTEXT_BUFFERS',
        {(0, None): warp_rc},
    ):
      for leading_shape in ((1,), (3,), (2, 3)):
        with self.subTest(leading_shape=leading_shape):
          depth_data = jnp.zeros(
              leading_shape + (width * height,), dtype=jnp.float32
          )
          depth = jax.jit(render_util.get_depth, static_argnums=(0, 1, 3))(
              rc, 0, depth_data, 5.0
          )

          self.assertEqual(depth.shape, leading_shape + (height, width, 1))

  def test_get_depth_vmap(self):
    nworld, width, height = 3, 4, 4
    warp_rc = _fake_render_context(1, width, height)
    rc = mock.MagicMock(spec=RenderContextPytree, key=0)
    depth_data = jnp.zeros((nworld, width * height), dtype=jnp.float32)

    with mock.patch.dict(
        'mujoco.mjx.warp.render_context._MJX_RENDER_CONTEXT_BUFFERS',
        {(0, None): warp_rc},
    ):
      depth = jax.jit(
          jax.vmap(render_util.get_depth, in_axes=(None, None, 0, None)),
          static_argnums=(0, 1, 3),
      )(rc, 0, depth_data, 5.0)

    self.assertEqual(depth.shape, (nworld, height, width, 1))

  def test_get_segmentation(self):
    width, height = 4, 4
    warp_rc = _fake_render_context(1, width, height)
    rc = mock.MagicMock(spec=RenderContextPytree, key=0)
    seg_data = jnp.arange(width * height, dtype=jnp.int32)

    with mock.patch.dict(
        'mujoco.mjx.warp.render_context._MJX_RENDER_CONTEXT_BUFFERS',
        {(0, None): warp_rc},
    ):
      segmentation = jax.jit(
          render_util.get_segmentation, static_argnums=(0, 1)
      )(rc, 0, seg_data)

    self.assertEqual(segmentation.shape, (height, width))
    np.testing.assert_array_equal(
        np.asarray(segmentation),
        np.arange(width * height, dtype=np.int32).reshape(height, width),
    )

  def test_get_segmentation_preserves_leading_dims(self):
    width, height = 4, 4
    warp_rc = _fake_render_context(1, width, height)
    rc = mock.MagicMock(spec=RenderContextPytree, key=0)

    with mock.patch.dict(
        'mujoco.mjx.warp.render_context._MJX_RENDER_CONTEXT_BUFFERS',
        {(0, None): warp_rc},
    ):
      for leading_shape in ((1,), (3,), (2, 3)):
        with self.subTest(leading_shape=leading_shape):
          seg_data = jnp.arange(
              np.prod(leading_shape) * width * height, dtype=jnp.int32
          ).reshape(leading_shape + (width * height,))
          segmentation = jax.jit(
              render_util.get_segmentation, static_argnums=(0, 1)
          )(rc, 0, seg_data)

          self.assertEqual(segmentation.shape, leading_shape + (height, width))

  def test_get_segmentation_vmap(self):
    nworld, width, height = 3, 4, 4
    warp_rc = _fake_render_context(1, width, height)
    rc = mock.MagicMock(spec=RenderContextPytree, key=0)
    seg_data = jnp.arange(nworld * width * height, dtype=jnp.int32).reshape(
        nworld, width * height
    )

    with mock.patch.dict(
        'mujoco.mjx.warp.render_context._MJX_RENDER_CONTEXT_BUFFERS',
        {(0, None): warp_rc},
    ):
      segmentation = jax.jit(
          jax.vmap(render_util.get_segmentation, in_axes=(None, None, 0)),
          static_argnums=(0, 1),
      )(rc, 0, seg_data)

    self.assertEqual(segmentation.shape, (nworld, height, width))

  def test_get_segmentation_raises_for_disabled_camera(self):
    width, height = 4, 4
    warp_rc = _fake_render_context(2, width, height, render_seg=[True, False])
    rc = mock.MagicMock(spec=RenderContextPytree, key=0)
    seg_data = jnp.arange(width * height, dtype=jnp.int32)

    with mock.patch.dict(
        'mujoco.mjx.warp.render_context._MJX_RENDER_CONTEXT_BUFFERS',
        {(0, None): warp_rc},
    ):
      with self.assertRaisesWithLiteralMatch(
          ValueError,
          'Camera 1 was not configured with segmentation rendering.',
      ):
        render_util.get_segmentation(rc, 1, seg_data)


if __name__ == '__main__':
  absltest.main()
