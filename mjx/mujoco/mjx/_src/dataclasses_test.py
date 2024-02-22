"""Tests custom dataclass."""

from absl.testing import absltest
import jax
from jax import numpy as jp
from mujoco.mjx._src import dataclasses
import numpy as np


class A(dataclasses.PyTreeNode):
  a: jax.Array
  b: jax.Array


class DataclassesTest(absltest.TestCase):

  def test_filter_k_arr(self):
    arr = jp.array([7, 6, 1, 10, 2, 3, 4, 5])
    mask = arr > 4
    res, fill_mask = jax.jit(dataclasses.filter_k, static_argnums=(2,))(
        arr, mask, k=5
    )
    np.testing.assert_array_equal(res[~fill_mask.astype(bool)], arr[arr > 4])

  def test_filter_k_tree(self):
    a = jp.array([7, 6, 1, 10, 2, 3, 4, 5])
    b = jp.zeros((a.shape[0], 3)) + jp.arange(a.shape[0])[:, None] + 1
    tree = A(a, b)
    mask = a > 4

    res_tree, fill_mask = jax.jit(dataclasses.filter_k, static_argnums=(2,))(
        tree, mask, k=5
    )
    np.testing.assert_array_equal(
        res_tree.a[~fill_mask.astype(bool)], tree.a[mask]
    )
    np.testing.assert_array_equal(
        res_tree.b[~fill_mask.astype(bool)], tree.b[mask]
    )

  def test_fill(self):
    a = jp.array([7, 6, 1, 10, 2, 3, 4, 5])
    b = jp.zeros((a.shape[0], 3)) + jp.arange(a.shape[0])[:, None] + 1
    tree = A(a, b)
    mask = a > 4

    res_tree, fill_mask = jax.jit(dataclasses.filter_k, static_argnums=(2,))(
        tree, mask, k=5
    )

    default = A(a=jp.array(-1), b=jp.array([-1, -1, -1]))
    res_tree = dataclasses.fill(res_tree, default, fill_mask)
    expected_a = np.concatenate([tree.a[mask], np.array([-1])])
    expected_b = np.concatenate([tree.b[mask], np.array([[-1, -1, -1]])])
    np.testing.assert_array_equal(res_tree.a, expected_a)
    np.testing.assert_array_equal(res_tree.b, expected_b)


if __name__ == "__main__":
  absltest.main()
