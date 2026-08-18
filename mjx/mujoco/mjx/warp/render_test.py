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
import tempfile

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


def _get_model_data_rc(xml, batch_size, render_seg=False):
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
      render_seg=render_seg,
      enabled_geom_groups=[0, 1, 2],
  )
  return mx, dx_batch, rc


class RenderTest(parameterized.TestCase):

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if mjxw.WARP_INSTALLED:
      cls.tempdir = tempfile.TemporaryDirectory()
      wp.config.kernel_cache_dir = cls.tempdir.name

  @classmethod
  def tearDownClass(cls):
    super().tearDownClass()
    if hasattr(cls, 'tempdir'):
      cls.tempdir.cleanup()

  def setUp(self):
    super().setUp()
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
    rgb_arr, depth_arr, dx_batch = jax.jit(mjx.render)(
        mx, dx_batch, rc.pytree()
    )

    rgb = np.asarray(rgb_arr)
    depth = np.asarray(depth_arr)

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
    out_batch = jax.tree.map(
        lambda x: x.reshape(-1, *x.shape[2:]) if x.size > 0 else x, out_batch
    )
    rgb = np.asarray(out_batch[0])
    depth = np.asarray(out_batch[1])

    np.testing.assert_array_equal(rgb, ref_rgb)
    np.testing.assert_array_equal(depth, ref_depth)
    self.assertGreater(np.count_nonzero(rgb), 0)
    self.assertNotEqual(np.unique(rgb).shape[0], 1)
    self.assertGreater(np.count_nonzero(depth), 0)
    self.assertNotEqual(np.unique(depth).shape[0], 1)

  @parameterized.product(
      xml=('humanoid/humanoid.xml',),
      batch_size=(1, 16),
  )
  def test_render_with_segmentation(self, xml: str, batch_size: int):
    """Tests MJX render pipeline with packed segmentation output."""
    self._maybe_skip()
    mx, dx_batch, rc = _get_model_data_rc(xml, batch_size, render_seg=True)

    dx_batch = jax.jit(mjx.refit_bvh)(mx, dx_batch, rc.pytree())
    rgb_arr, depth_arr, seg_arr, dx_batch = jax.jit(
        mjx.render_with_segmentation
    )(mx, dx_batch, rc.pytree())

    rgb = np.asarray(rgb_arr)
    depth = np.asarray(depth_arr)
    seg = np.asarray(seg_arr)

    self.assertGreater(np.count_nonzero(rgb), 0)
    self.assertGreater(np.count_nonzero(depth), 0)
    self.assertTrue(np.any(seg[..., 0] != -1))
    self.assertGreater(np.unique(seg[..., 0]).shape[0], 1)

    unpacked_seg = jax.vmap(mjx.get_segmentation, in_axes=(None, None, 0))(
        rc.pytree(), 0, seg_arr
    )
    unpacked_seg = np.asarray(unpacked_seg)
    width, height = rc._default.cam_res.numpy()[
        0
    ]  # pylint: disable=protected-access
    seg_adr = int(
        rc._default.seg_adr.numpy()[0]  # pylint: disable=protected-access
    )
    # seg shape: (batch, total_pixels, 2); extract objid channel
    expected_seg = seg[:, seg_adr : seg_adr + width * height, 0].reshape(
        batch_size, height, width
    )
    np.testing.assert_array_equal(unpacked_seg, expected_seg)

  def test_render_with_segmentation_raises_when_disabled(self):
    """Tests render_with_segmentation rejects contexts without seg output."""
    self._maybe_skip()
    mx, dx_batch, rc = _get_model_data_rc(
        'humanoid/humanoid.xml', 1, render_seg=False
    )

    dx_batch = jax.jit(mjx.refit_bvh)(mx, dx_batch, rc.pytree())
    with self.assertRaisesWithLiteralMatch(
        ValueError,
        'Render context was not configured with segmentation rendering. '
        'Pass render_seg=True or enable it for at least one camera in '
        'create_render_context.',
    ):
      jax.jit(mjx.render_with_segmentation)(mx, dx_batch, rc.pytree())

  @parameterized.product(
      xml=('humanoid/humanoid.xml',),
      batch_size=(4, 16),
  )
  def test_render_with_segmentation_nested_vmap(
      self, xml: str, batch_size: int
  ):
    """Tests MJX render_with_segmentation with nested vmap."""
    self._maybe_skip()
    mx, dx_batch, rc = _get_model_data_rc(xml, batch_size, render_seg=True)

    def inner(mx, dx, rc):
      dx = jax.vmap(bvh.refit_bvh, in_axes=(None, 0, None))(mx, dx, rc)
      out = jax.vmap(render.render_with_segmentation, in_axes=(None, 0, None))(
          mx, dx, rc
      )
      return out

    dx_batch = jax.vmap(bvh.refit_bvh, in_axes=(None, 0, None))(
        mx, dx_batch, rc.pytree()
    )
    ref = jax.vmap(render.render_with_segmentation, in_axes=(None, 0, None))(
        mx, dx_batch, rc.pytree()
    )
    ref_rgb = np.asarray(ref[0])
    ref_depth = np.asarray(ref[1])
    ref_seg = np.asarray(ref[2])

    def _reshape_batched(x):
      if x.shape[0] == batch_size:
        return x.reshape(2, batch_size // 2, *x.shape[1:])
      return x

    dx_2d = jax.tree.map(_reshape_batched, dx_batch)

    out_batch = jax.vmap(inner, in_axes=(None, 0, None))(mx, dx_2d, rc.pytree())
    out_batch = jax.tree.map(
        lambda x: x.reshape(-1, *x.shape[2:]) if x.size > 0 else x, out_batch
    )
    rgb = np.asarray(out_batch[0])
    depth = np.asarray(out_batch[1])
    seg = np.asarray(out_batch[2])

    np.testing.assert_array_equal(rgb, ref_rgb)
    np.testing.assert_array_equal(depth, ref_depth)
    np.testing.assert_array_equal(seg, ref_seg)
    self.assertTrue(np.any(seg[..., 0] != -1))

  def test_sliding_box_poses(self):
    """Tests that refit_bvh executes before render for multiple poses in a single JIT."""
    self._maybe_skip()
    xml = """\
    <mujoco>
      <option gravity="0 0 0"/>
      <asset>
        <material name="mat" rgba="1 1 1 1"/>
      </asset>
      <worldbody>
        <camera pos="0 0 1" resolution="64 64"
          sensorsize="0.036 0.036" focal="0.012 0.012"/>
        <geom type="plane" size="2 2 0.1" material="mat"/>
        <body pos="0 0 0.1" mocap="true">
          <geom type="box" size="0.1 0.1 0.1" material="mat"/>
        </body>
        <body pos="0 0 5">
          <joint type="free"/>
          <geom type="sphere" size="0.01" group="3"
            contype="0" conaffinity="0"/>
        </body>
      </worldbody>
    </mujoco>
    """
    m = mujoco.MjModel.from_xml_string(xml)
    mx = mjx.put_model(m, impl='warp')
    dx = mjx.put_data(m, mujoco.MjData(m), impl='warp')
    rc = mjx.create_render_context(
        mjm=m, nworld=1, cam_res=(64, 64), render_rgb=True, render_depth=True
    )

    x_positions = (-0.8, -0.4, 0.0, 0.4, 0.8)

    def step_and_render(d, x_pos):
      mocap_pos = d.mocap_pos.at[0, 0].set(x_pos)
      d = forward.forward(mx, d.replace(mocap_pos=mocap_pos))
      d = mjx.refit_bvh(mx, d, rc.pytree())
      _, depth, d = mjx.render(mx, d, rc.pytree())
      return d, depth

    # 1. Step-by-step execution.
    ref_depths = []
    d_curr = dx
    for x in x_positions:
      d_curr, depth = jax.jit(step_and_render)(d_curr, x)
      ref_depths.append(np.asarray(depth))

    # 2. Multi-step rollout in a single compiled jax.jit via jax.lax.scan.
    def rollout_scan(d0):
      def scan_fn(d, x):
        d, depth = step_and_render(d, x)
        return d, depth

      _, scan_depths = jax.lax.scan(
          scan_fn, d0, np.array(x_positions, dtype=np.float32)
      )
      return scan_depths

    scanned_depths = [np.asarray(d) for d in jax.jit(rollout_scan)(dx)]

    # 3. Explicitly unrolled sequence in a single compiled jax.jit.
    def rollout_unroll(d0):
      depths = []
      d = d0
      for x in x_positions:
        mocap_pos = d.mocap_pos.at[0, 0].set(x)
        d = forward.forward(mx, d.replace(mocap_pos=mocap_pos))
        d = mjx.refit_bvh(mx, d, rc.pytree())
        _, depth, d = mjx.render(mx, d, rc.pytree())
        depths.append(depth)
      return depths

    unrolled_depths = [np.asarray(d) for d in jax.jit(rollout_unroll)(dx)]

    # Assert exact equality across all frames for both scan and unroll.
    for i, (want, scan_got, unroll_got) in enumerate(
        zip(ref_depths, scanned_depths, unrolled_depths)
    ):
      np.testing.assert_array_equal(
          scan_got, want, err_msg=f'Scan mismatch at frame {i}'
      )
      np.testing.assert_array_equal(
          unroll_got, want, err_msg=f'Unroll mismatch at frame {i}'
      )

    del rc


class RenderContextGarbageCollectionTest(absltest.TestCase):
  """Tests that RenderContext cleans up buffers on deletion."""

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if mjxw.WARP_INSTALLED:
      cls.tempdir = tempfile.TemporaryDirectory()
      wp.config.kernel_cache_dir = cls.tempdir.name

  @classmethod
  def tearDownClass(cls):
    super().tearDownClass()
    if hasattr(cls, 'tempdir'):
      cls.tempdir.cleanup()

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
