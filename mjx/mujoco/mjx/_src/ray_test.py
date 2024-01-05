# Copyright 2023 DeepMind Technologies Limited
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
"""Tests for ray functions."""

from absl.testing import absltest
import jax
from jax import numpy as jp
import mujoco
from mujoco import mjx
from mujoco.mjx._src import test_util
import numpy as np

# tolerance for difference between MuJoCo and MJX ray calculations - mostly
# due to float precision
_TOLERANCE = 5e-5


def _assert_eq(a, b, name):
  tol = _TOLERANCE * 10  # avoid test noise
  err_msg = f'mismatch: {name}'
  np.testing.assert_allclose(a, b, err_msg=err_msg, atol=tol, rtol=tol)


class RayTest(absltest.TestCase):

  def test_ray_nothing(self):
    """Tests that MJX ray returns -1 when nothing is hit."""
    m = test_util.load_test_file('ray.xml')
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    mx, dx = mjx.put_model(m), mjx.put_data(m, d)

    pnt, vec = jp.array([12.146, 1.865, 3.895]), jp.array([0, 0, -1.0])
    geomid, dist = jax.jit(mjx.ray)(mx, dx, pnt, vec)
    _assert_eq(geomid, jp.array([-1]), 'geom_id')
    _assert_eq(dist, jp.array([-1]), 'dist')

  def test_ray_plane(self):
    """Tests MJX ray<>plane matches MuJoCo."""
    m = test_util.load_test_file('ray.xml')
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    mx, dx = mjx.put_model(m), mjx.put_data(m, d)

    # looking down at a slight angle
    pnt, vec = jp.array([2, 1, 3.0]), jp.array([0.1, 0.2, -1.0])
    vec /= jp.linalg.norm(vec)
    geomid, dist = jax.jit(mjx.ray)(mx, dx, pnt, vec)
    _assert_eq(geomid, jp.array([0]), 'geom_id')
    pnt, vec, unused = np.array(pnt), np.array(vec), np.zeros(1, dtype=np.int32)
    mj_dist = mujoco.mj_ray(m, d, pnt, vec, None, 1, -1, unused)
    _assert_eq(dist, mj_dist, 'dist')

    # looking on wrong side of plane
    pnt = jp.array([0, 0, -0.5])
    geomid, dist = jax.jit(mjx.ray)(mx, dx, pnt, vec)
    _assert_eq(geomid, jp.array([-1]), 'geom_id')
    _assert_eq(dist, jp.array([-1]), 'dist')

  def test_ray_sphere(self):
    """Tests MJX ray<>sphere matches MuJoCo."""
    m = test_util.load_test_file('ray.xml')
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    mx, dx = mjx.put_model(m), mjx.put_data(m, d)

    # looking down at sphere at a slight angle
    pnt, vec = jp.array([0, 0, 1.6]), jp.array([0.1, 0.2, -1.0])
    vec /= jp.linalg.norm(vec)
    geomid, dist = jax.jit(mjx.ray)(mx, dx, pnt, vec)
    _assert_eq(geomid, jp.array([1]), 'geom_id')
    pnt, vec, unused = np.array(pnt), np.array(vec), np.zeros(1, dtype=np.int32)
    mj_dist = mujoco.mj_ray(m, d, pnt, vec, None, 1, -1, unused)
    _assert_eq(dist, mj_dist, 'dist')

  def test_ray_capsule(self):
    """Tests MJX ray<>capsule matches MuJoCo."""
    m = test_util.load_test_file('ray.xml')
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    mx, dx = mjx.put_model(m), mjx.put_data(m, d)

    # looking down at capsule at a slight angle
    pnt, vec = jp.array([0.5, 1, 1.6]), jp.array([0, 0.05, -1.0])
    vec /= jp.linalg.norm(vec)
    geomid, dist = jax.jit(mjx.ray)(mx, dx, pnt, vec)
    _assert_eq(geomid, jp.array([2]), 'geom_id')
    pnt, vec, unused = np.array(pnt), np.array(vec), np.zeros(1, dtype=np.int32)
    mj_dist = mujoco.mj_ray(m, d, pnt, vec, None, 1, -1, unused)
    _assert_eq(dist, mj_dist, 'dist')

    # looking up at capsule from below
    pnt, vec = jp.array([-0.5, 1, 0.05]), jp.array([0, 0.05, 1.0])
    vec /= jp.linalg.norm(vec)
    geomid, dist = jax.jit(mjx.ray)(mx, dx, pnt, vec)
    _assert_eq(geomid, jp.array([2]), 'geom_id')
    pnt, vec, unused = np.array(pnt), np.array(vec), np.zeros(1, dtype=np.int32)
    mj_dist = mujoco.mj_ray(m, d, pnt, vec, None, 1, -1, unused)
    _assert_eq(dist, mj_dist, 'dist')

    # looking at cylinder of capsule from the side
    pnt, vec = jp.array([0, 1, 0.75]), jp.array([1, 0, 0])
    vec /= jp.linalg.norm(vec)
    geomid, dist = jax.jit(mjx.ray)(mx, dx, pnt, vec)
    _assert_eq(geomid, jp.array([2]), 'geom_id')
    pnt, vec, unused = np.array(pnt), np.array(vec), np.zeros(1, dtype=np.int32)
    mj_dist = mujoco.mj_ray(m, d, pnt, vec, None, 1, -1, unused)
    _assert_eq(dist, mj_dist, 'dist')

  def test_ray_box(self):
    """Tests MJX ray<>box matches MuJoCo."""
    m = test_util.load_test_file('ray.xml')
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    mx, dx = mjx.put_model(m), mjx.put_data(m, d)

    # looking down at box at a slight angle
    pnt, vec = jp.array([1, 0, 1.6]), jp.array([0, 0.05, -1.0])
    vec /= jp.linalg.norm(vec)
    geomid, dist = jax.jit(mjx.ray)(mx, dx, pnt, vec)
    _assert_eq(geomid, jp.array([3]), 'geom_id')
    pnt, vec, unused = np.array(pnt), np.array(vec), np.zeros(1, dtype=np.int32)
    mj_dist = mujoco.mj_ray(m, d, pnt, vec, None, 1, -1, unused)
    _assert_eq(dist, mj_dist, 'dist')

    # looking up at box from below
    pnt, vec = jp.array([1, 0, 0.05]), jp.array([0, 0.05, 1.0])
    vec /= jp.linalg.norm(vec)
    geomid, dist = jax.jit(mjx.ray)(mx, dx, pnt, vec)
    _assert_eq(geomid, jp.array([3]), 'geom_id')
    pnt, vec, unused = np.array(pnt), np.array(vec), np.zeros(1, dtype=np.int32)
    mj_dist = mujoco.mj_ray(m, d, pnt, vec, None, 1, -1, unused)
    _assert_eq(dist, mj_dist, 'dist')


if __name__ == '__main__':
  absltest.main()
