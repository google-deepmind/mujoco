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
"""Tests for render_jax module."""

import os
import tempfile

from absl.testing import absltest
import jax
import jax.numpy as jnp
import mujoco
from mujoco.mjx._src import io
import mujoco.mjx.warp as mjxw
import numpy as np

try:
    from mujoco.mjx.warp import render_jax  # pylint: disable=g-import-not-at-top
except ImportError:
    render_jax = None

try:
    import mujoco_warp as mjwarp  # pylint: disable=g-import-not-at-top
    # Work around version mismatch: the standalone mujoco_warp may detect
    # a dev MuJoCo version via metadata but the build may not have all
    # bleeding-edge attributes. Force BLEEDING_EDGE_MUJOCO=False so that
    # put_model falls back to reading flexedge fields from MjData.
    try:
        import mujoco_warp._src.io as _mjwarp_io
        if _mjwarp_io.BLEEDING_EDGE_MUJOCO:
            import mujoco as _mj
            _mjm_test = _mj.MjModel.from_xml_string('<mujoco/>')
            if not hasattr(_mjm_test, 'flexedge_J_rownnz'):
                _mjwarp_io.BLEEDING_EDGE_MUJOCO = False
    except (ImportError, AttributeError):
        pass
except ImportError:
    try:
        from mujoco.mjx.third_party import mujoco_warp as mjwarp  # pylint: disable=g-import-not-at-top
    except ImportError:
        mjwarp = None

try:
    import warp as wp  # pylint: disable=g-import-not-at-top
except ImportError:
    wp = None

_FORCE_TEST = os.environ.get('MJX_WARP_FORCE_TEST', '0') == '1'

_XML = """
<mujoco>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="5 5 0.1" rgba="0.8 0.8 0.8 1"/>
    <body pos="0 0 0.5">
      <geom type="sphere" size="0.2" rgba="1 0 0 1"/>
    </body>
    <camera name="cam0" pos="0 -2 1" xyaxes="1 0 0 0 0.5 1"/>
  </worldbody>
</mujoco>
"""

_NWORLDS = 4
_RENDER_WIDTH = 32
_RENDER_HEIGHT = 32


def _skip_if_no_gpu(test_case):
    """Skip test if no CUDA GPU or required packages are missing."""
    if not _FORCE_TEST:
        if not mjxw.WARP_INSTALLED:
            test_case.skipTest('Warp not installed.')
        if not io.has_cuda_gpu_device():
            test_case.skipTest('No CUDA GPU device available.')
    if render_jax is None:
        test_case.skipTest('render_jax module not importable.')
    if mjwarp is None:
        test_case.skipTest('mujoco_warp not importable.')


def _create_scene():
    """Create a simple test scene and return (mjm, m, d, jax_inputs)."""
    mjm = mujoco.MjModel.from_xml_string(_XML)
    mjd = mujoco.MjData(mjm)
    mujoco.mj_forward(mjm, mjd)

    m = mjwarp.put_model(mjm)
    d = mjwarp.put_data(mjm, mjd, nworld=_NWORLDS)
    mjwarp.forward(m, d)

    # Extract current kinematic state as JAX arrays.
    jax_inputs = {
        'geom_xpos': jnp.array(d.geom_xpos.numpy()),
        'geom_xmat': jnp.array(d.geom_xmat.numpy()),
        'cam_xpos': jnp.array(d.cam_xpos.numpy()),
        'cam_xmat': jnp.array(d.cam_xmat.numpy()),
        'light_xpos': jnp.array(d.light_xpos.numpy()),
        'light_xdir': jnp.array(d.light_xdir.numpy()),
    }

    return mjm, m, d, jax_inputs


class RenderJaxTest(absltest.TestCase):

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        if wp is not None:
            cls.tempdir = tempfile.TemporaryDirectory()
            wp.config.kernel_cache_dir = cls.tempdir.name

    @classmethod
    def tearDownClass(cls):
        super().tearDownClass()
        if hasattr(cls, 'tempdir'):
            cls.tempdir.cleanup()

    def test_create_render_fn(self):
        """Tests that create_mjx_render_fn returns a callable and info dict."""
        _skip_if_no_gpu(self)
        mjm, m, d, _ = _create_scene()

        render_fn, info = render_jax.create_mjx_render_fn(
            mjm, m, d, cam_res=(_RENDER_WIDTH, _RENDER_HEIGHT),
        )

        self.assertTrue(callable(render_fn))
        self.assertIsInstance(info, dict)
        self.assertEqual(info['nworld'], _NWORLDS)
        self.assertEqual(info['cam_res'], (_RENDER_WIDTH, _RENDER_HEIGHT))
        self.assertEqual(info['ngeom'], mjm.ngeom)
        self.assertEqual(info['ncam'], mjm.ncam)
        self.assertEqual(info['nlight'], mjm.nlight)
        self.assertIsNotNone(info['rgb_shape'])
        self.assertEqual(info['rgb_shape'][0], _NWORLDS)
        self.assertIsNone(info['depth_shape'])
        self.assertGreater(info['total_pixels'], 0)

    def test_render_produces_nonzero_output(self):
        """Tests that rendering produces non-zero pixel data."""
        _skip_if_no_gpu(self)
        mjm, m, d, jax_inputs = _create_scene()

        render_fn, info = render_jax.create_mjx_render_fn(
            mjm, m, d, cam_res=(_RENDER_WIDTH, _RENDER_HEIGHT),
        )

        rgb = render_fn(**jax_inputs)
        rgb_np = np.array(rgb)

        self.assertEqual(rgb_np.shape, info['rgb_shape'])
        self.assertEqual(rgb_np.dtype, np.uint32)
        self.assertGreater(
            np.count_nonzero(rgb_np), 0,
            'All pixels are zero -- rendering failed',
        )

    def test_render_inside_jit(self):
        """Tests that the render function works inside jax.jit."""
        _skip_if_no_gpu(self)
        mjm, m, d, jax_inputs = _create_scene()

        render_fn, info = render_jax.create_mjx_render_fn(
            mjm, m, d, cam_res=(_RENDER_WIDTH, _RENDER_HEIGHT),
        )

        @jax.jit
        def jitted_render(gxp, gxm, cxp, cxm, lxp, lxd):
            return render_fn(gxp, gxm, cxp, cxm, lxp, lxd)

        rgb = jitted_render(
            jax_inputs['geom_xpos'], jax_inputs['geom_xmat'],
            jax_inputs['cam_xpos'], jax_inputs['cam_xmat'],
            jax_inputs['light_xpos'], jax_inputs['light_xdir'],
        )
        jax.block_until_ready(rgb)
        rgb_np = np.array(rgb)

        self.assertEqual(rgb_np.shape, info['rgb_shape'])
        self.assertGreater(
            np.count_nonzero(rgb_np), 0,
            'All pixels are zero -- JIT render failed',
        )

    def test_render_inside_lax_scan(self):
        """Tests that the render function works inside jax.lax.scan."""
        _skip_if_no_gpu(self)
        mjm, m, d, jax_inputs = _create_scene()

        render_fn, info = render_jax.create_mjx_render_fn(
            mjm, m, d, cam_res=(_RENDER_WIDTH, _RENDER_HEIGHT),
        )

        scan_length = 3

        @jax.jit
        def scan_render(gxp, gxm, cxp, cxm, lxp, lxd):
            def body(carry, _):
                rgb = render_fn(gxp, gxm, cxp, cxm, lxp, lxd)
                return carry, rgb

            _, rgbs = jax.lax.scan(body, None, length=scan_length)
            return rgbs

        rgbs = scan_render(
            jax_inputs['geom_xpos'], jax_inputs['geom_xmat'],
            jax_inputs['cam_xpos'], jax_inputs['cam_xmat'],
            jax_inputs['light_xpos'], jax_inputs['light_xdir'],
        )
        jax.block_until_ready(rgbs)
        rgbs_np = np.array(rgbs)

        expected_shape = (scan_length,) + info['rgb_shape']
        self.assertEqual(rgbs_np.shape, expected_shape)
        self.assertGreater(
            np.count_nonzero(rgbs_np), 0,
            'All pixels are zero -- scan render failed',
        )

    def test_unpack_rgb(self):
        """Tests RGB unpacking with known packed values."""
        _skip_if_no_gpu(self)

        # Pack known RGB values: R=255, G=128, B=0 -> 0x00FF8000
        packed_val = (255 << 16) | (128 << 8) | 0
        nworld, h, w = 2, 2, 2
        total_pixels = h * w
        rgb_packed = jnp.full((nworld, total_pixels), packed_val, dtype=jnp.uint32)

        result = render_jax.unpack_rgb(rgb_packed, h, w)

        self.assertEqual(result.shape, (nworld, h, w, 3))
        self.assertEqual(result.dtype, jnp.float32)
        # R channel = 255/255 = 1.0
        np.testing.assert_allclose(result[..., 0], 1.0, atol=1e-6)
        # G channel = 128/255 ~ 0.502
        np.testing.assert_allclose(result[..., 1], 128 / 255.0, atol=1e-6)
        # B channel = 0/255 = 0.0
        np.testing.assert_allclose(result[..., 2], 0.0, atol=1e-6)

    def test_unpack_rgb_black_and_white(self):
        """Tests RGB unpacking with black (0) and white (0xFFFFFF) pixels."""
        _skip_if_no_gpu(self)

        h, w = 1, 2
        black = jnp.uint32(0)
        white = jnp.uint32((255 << 16) | (255 << 8) | 255)
        rgb_packed = jnp.array([[black, white]], dtype=jnp.uint32)

        result = render_jax.unpack_rgb(rgb_packed, h, w)

        self.assertEqual(result.shape, (1, h, w, 3))
        # Black pixel
        np.testing.assert_allclose(result[0, 0, 0, :], [0.0, 0.0, 0.0], atol=1e-6)
        # White pixel
        np.testing.assert_allclose(result[0, 0, 1, :], [1.0, 1.0, 1.0], atol=1e-6)

    def test_unpack_grayscale(self):
        """Tests grayscale unpacking with known packed values."""
        _skip_if_no_gpu(self)

        # Pure white: R=G=B=255 -> grayscale should be 1.0
        white = (255 << 16) | (255 << 8) | 255
        nworld, h, w = 1, 2, 2
        total_pixels = h * w
        rgb_packed = jnp.full(
            (nworld, total_pixels), white, dtype=jnp.uint32
        )

        result = render_jax.unpack_grayscale(rgb_packed, h, w)

        self.assertEqual(result.shape, (nworld, h, w, 1))
        self.assertEqual(result.dtype, jnp.float32)
        # White: (0.299*255 + 0.587*255 + 0.114*255) / 255 = 1.0
        np.testing.assert_allclose(result, 1.0, atol=1e-5)

    def test_unpack_grayscale_known_value(self):
        """Tests grayscale with a known non-trivial color."""
        _skip_if_no_gpu(self)

        # R=100, G=150, B=200
        packed = (100 << 16) | (150 << 8) | 200
        rgb_packed = jnp.array([[packed]], dtype=jnp.uint32)

        result = render_jax.unpack_grayscale(rgb_packed, 1, 1)

        expected = (0.299 * 100 + 0.587 * 150 + 0.114 * 200) / 255.0
        np.testing.assert_allclose(result[0, 0, 0, 0], expected, atol=1e-5)

    def test_render_with_depth(self):
        """Tests render with depth output enabled."""
        _skip_if_no_gpu(self)
        mjm, m, d, jax_inputs = _create_scene()

        render_fn, info = render_jax.create_mjx_render_fn(
            mjm, m, d,
            cam_res=(_RENDER_WIDTH, _RENDER_HEIGHT),
            render_depth=True,
        )

        self.assertIsNotNone(info['depth_shape'])
        self.assertEqual(info['depth_shape'][0], _NWORLDS)

        rgb, depth = render_fn(**jax_inputs)
        rgb_np = np.array(rgb)
        depth_np = np.array(depth)

        self.assertEqual(rgb_np.shape, info['rgb_shape'])
        self.assertEqual(depth_np.shape, info['depth_shape'])
        self.assertEqual(rgb_np.dtype, np.uint32)
        self.assertEqual(depth_np.dtype, np.float32)
        self.assertGreater(
            np.count_nonzero(rgb_np), 0,
            'All RGB pixels are zero -- depth render failed',
        )


if __name__ == '__main__':
    absltest.main()
