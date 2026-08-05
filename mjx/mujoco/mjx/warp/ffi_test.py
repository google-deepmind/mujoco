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

from unittest import mock

from absl.testing import absltest

from mujoco.mjx.warp import ffi


class FfiTest(absltest.TestCase):

  def test_jax_callable_variadic_tuple_cache(self):
    def func_a(values: tuple[int, ...], output: int):
      del values, output

    def func_b(values: tuple[int, ...], output: int):
      del values, output

    # Count callback registrations without creating real JAX FFI targets.
    def create_callable(*args, **kwargs):
      del args, kwargs
      return mock.Mock()

    with mock.patch.object(ffi, '_JAX_CALLABLE_VARIADIC_TUPLE_REGISTRY', {}):
      with mock.patch.object(
          ffi.warp_ffi, 'jax_callable', side_effect=create_callable
      ) as jax_callable:
        wrapper_a = ffi.jax_callable_variadic_tuple(func_a)
        wrapper_a_again = ffi.jax_callable_variadic_tuple(func_a)
        wrapper_b = ffi.jax_callable_variadic_tuple(func_b)

        # Equivalent traces of the same callable reuse one callback.
        wrapper_a((1, 2))
        wrapper_a_again((3, 4))
        self.assertEqual(jax_callable.call_count, 1)

        # Distinct shim functions with the same signature do not share a target.
        wrapper_b((5, 6))
        self.assertEqual(jax_callable.call_count, 2)

        # A different tuple arity changes the signature and PyTree cache key.
        wrapper_b((1, 2, 3))
        self.assertEqual(jax_callable.call_count, 3)


if __name__ == '__main__':
  absltest.main()
