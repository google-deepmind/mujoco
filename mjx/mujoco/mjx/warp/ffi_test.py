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
"""Tests for MJX-Warp FFI helpers."""

import os
import tempfile

# Two devices are needed to build a sharded mesh; fake them on CPU so this
# runs under ordinary CI. Must precede the first JAX import.
os.environ.setdefault(
    'XLA_FLAGS',
    os.environ.get('XLA_FLAGS', '') + ' --xla_force_host_platform_device_count=2',
)

from absl.testing import absltest  # pylint: disable=g-import-not-at-top
import jax
from jax import numpy as jp
from jax.sharding import Mesh
from jax.sharding import PartitionSpec as P
import mujoco.mjx.warp as mjxw
import numpy as np

if mjxw.WARP_INSTALLED:
  from mujoco.mjx.third_party.warp._src.jax import ffi as warp_ffi  # pylint: disable=g-import-not-at-top
  from mujoco.mjx.warp import ffi  # pylint: disable=g-import-not-at-top
  from mujoco.mjx.warp import warp as wp  # pylint: disable=g-import-not-at-top


class FfiVmaTest(absltest.TestCase):
  """Varying-axis metadata must survive an FFI call (mujoco #3426)."""

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if not mjxw.WARP_INSTALLED:
      return
    cls.tempdir = tempfile.TemporaryDirectory()
    wp.config.kernel_cache_dir = cls.tempdir.name

    def copy(
        x: wp.array(dtype=wp.float32, ndim=1),
        out: wp.array(dtype=wp.float32, ndim=1),
    ):
      wp.copy(out, x)

    @wp.kernel
    def add_kernel(
        x: wp.array(dtype=wp.float32),
        y: wp.array(dtype=wp.float32),
        out: wp.array(dtype=wp.float32),
    ):
      i = wp.tid()
      out[i] = x[i] + y[i]

    def add(
        x: wp.array(dtype=wp.float32, ndim=1),
        y: wp.array(dtype=wp.float32, ndim=1),
        out: wp.array(dtype=wp.float32, ndim=1),
    ):
      wp.launch(add_kernel, dim=x.shape[0], inputs=[x, y], outputs=[out])

    # staticmethod: a bare function on the class would bind `self` as the
    # first FFI input.
    graph_mode = warp_ffi.JaxCallableGraphMode.NONE
    cls.copy_ffi = staticmethod(
        ffi.jax_callable_variadic_tuple(copy, graph_mode=graph_mode)
    )
    cls.add_ffi = staticmethod(
        ffi.jax_callable_variadic_tuple(add, graph_mode=graph_mode)
    )

  @classmethod
  def tearDownClass(cls):
    super().tearDownClass()
    if hasattr(cls, 'tempdir'):
      cls.tempdir.cleanup()

  def setUp(self):
    super().setUp()
    if not mjxw.WARP_INSTALLED:
      self.skipTest('Warp not installed.')
    if not hasattr(jax.lax, 'pcast'):
      self.skipTest('JAX version does not support manual-axis casting.')
    if len(jax.devices()) < 2:
      self.skipTest('Test requires at least two JAX devices.')

    self.mesh = Mesh(np.array(jax.devices()[:2]), ('replicas',))

  def test_single_call_preserves_varying_axes(self):
    """A lone FFI call must not silently downgrade varying data."""
    observed = {}

    @jax.shard_map(
        mesh=self.mesh,
        in_specs=P('replicas'),
        out_specs=P('replicas'),
        check_vma=True,
    )
    def run(x):
      out = self.copy_ffi(x)[0]
      observed['out'] = jax.typeof(out).manual_axis_type.varying
      return out

    x = jp.arange(4, dtype=jp.float32)
    np.testing.assert_array_equal(run(x), x)
    # Asserts the mechanism, not a downstream symptom: without the fix this
    # is frozenset() while the call still returns the correct numbers.
    self.assertEqual(observed['out'], frozenset({'replicas'}))

  def test_varying_axes_survive_scan_carry(self):
    """The originally reported failure: scan carry types must match."""

    @jax.shard_map(
        mesh=self.mesh,
        in_specs=P('replicas'),
        out_specs=P('replicas'),
        check_vma=True,
    )
    def scan_copy(x):
      def body(carry, _):
        return self.copy_ffi(carry)[0], None

      return jax.lax.scan(body, x, None, length=2)[0]

    x = jp.arange(4, dtype=jp.float32)
    np.testing.assert_array_equal(scan_copy(x), x)

  def test_replicated_input_stays_replicated(self):
    """With no varying inputs there is nothing to propagate."""
    observed = {}

    @jax.shard_map(
        mesh=self.mesh, in_specs=P(), out_specs=P(), check_vma=True
    )
    def run(x):
      out = self.copy_ffi(x)[0]
      observed['out'] = jax.typeof(out).manual_axis_type.varying
      return out

    x = jp.arange(4, dtype=jp.float32)
    np.testing.assert_array_equal(run(x), x)
    self.assertEqual(observed['out'], frozenset())

  def test_mixed_inputs_take_union_of_varying_axes(self):
    """One varying and one replicated input must yield a varying output."""
    observed = {}

    @jax.shard_map(
        mesh=self.mesh,
        in_specs=(P('replicas'), P()),
        out_specs=P('replicas'),
        check_vma=True,
    )
    def run(x, y):
      out = self.add_ffi(x, y)[0]
      observed['x'] = jax.typeof(x).manual_axis_type.varying
      observed['y'] = jax.typeof(y).manual_axis_type.varying
      observed['out'] = jax.typeof(out).manual_axis_type.varying
      return out

    x = jp.arange(4, dtype=jp.float32)
    y = jp.ones(2, dtype=jp.float32)
    np.testing.assert_array_equal(run(x, y), x + 1.0)
    self.assertEqual(observed['x'], frozenset({'replicas'}))
    self.assertEqual(observed['y'], frozenset())
    self.assertEqual(observed['out'], frozenset({'replicas'}))


if __name__ == '__main__':
  absltest.main()
