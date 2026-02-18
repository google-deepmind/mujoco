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
from absl.testing import parameterized
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


class RayTest(parameterized.TestCase):

  def test_ray_nothing(self):
    """Tests that MJX ray returns -1 when nothing is hit."""
    m = test_util.load_test_file('ray.xml')
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    mx, dx = mjx.put_model(m), mjx.put_data(m, d)

    pnt, vec = jp.array([12.146, 1.865, 3.895]), jp.array([0, 0, -1.0])
    dist, geomid = jax.jit(mjx.ray)(mx, dx, pnt, vec)
    _assert_eq(geomid, -1, 'geom_id')
    _assert_eq(dist, -1, 'dist')

  def test_ray_plane(self):
    """Tests MJX ray<>plane matches MuJoCo."""
    m = test_util.load_test_file('ray.xml')
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    mx, dx = mjx.put_model(m), mjx.put_data(m, d)

    # looking down at a slight angle
    pnt, vec = jp.array([2, 1, 3.0]), jp.array([0.1, 0.2, -1.0])
    vec /= jp.linalg.norm(vec)
    dist, geomid = jax.jit(mjx.ray)(mx, dx, pnt, vec)
    _assert_eq(geomid, 0, 'geom_id')
    pnt, vec, unused = np.array(pnt), np.array(vec), np.zeros(1, dtype=np.int32)
    mj_dist = mujoco.mj_ray(m, d, pnt, vec, None, 1, -1, unused)
    _assert_eq(dist, mj_dist, 'dist')

    # looking on wrong side of plane
    pnt = jp.array([0, 0, -0.5])
    dist, geomid = jax.jit(mjx.ray)(mx, dx, pnt, vec)
    _assert_eq(geomid, -1, 'geom_id')
    _assert_eq(dist, -1, 'dist')

  def test_ray_sphere(self):
    """Tests MJX ray<>sphere matches MuJoCo."""
    m = test_util.load_test_file('ray.xml')
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    mx, dx = mjx.put_model(m), mjx.put_data(m, d)

    # looking down at sphere at a slight angle
    pnt, vec = jp.array([0, 0, 1.6]), jp.array([0.1, 0.2, -1.0])
    vec /= jp.linalg.norm(vec)
    dist, geomid = jax.jit(mjx.ray)(mx, dx, pnt, vec)
    _assert_eq(geomid, 1, 'geom_id')
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
    dist, geomid = jax.jit(mjx.ray)(mx, dx, pnt, vec)
    _assert_eq(geomid, 2, 'geom_id')
    pnt, vec, unused = np.array(pnt), np.array(vec), np.zeros(1, dtype=np.int32)
    mj_dist = mujoco.mj_ray(m, d, pnt, vec, None, 1, -1, unused)
    _assert_eq(dist, mj_dist, 'dist')

    # looking up at capsule from below
    pnt, vec = jp.array([-0.5, 1, 0.05]), jp.array([0, 0.05, 1.0])
    vec /= jp.linalg.norm(vec)
    dist, geomid = jax.jit(mjx.ray)(mx, dx, pnt, vec)
    _assert_eq(geomid, 2, 'geom_id')
    pnt, vec, unused = np.array(pnt), np.array(vec), np.zeros(1, dtype=np.int32)
    mj_dist = mujoco.mj_ray(m, d, pnt, vec, None, 1, -1, unused)
    _assert_eq(dist, mj_dist, 'dist')

    # looking at cylinder of capsule from the side
    pnt, vec = jp.array([0, 1, 0.75]), jp.array([1, 0, 0])
    vec /= jp.linalg.norm(vec)
    dist, geomid = jax.jit(mjx.ray)(mx, dx, pnt, vec)
    _assert_eq(geomid, 2, 'geom_id')
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
    dist, geomid = jax.jit(mjx.ray)(mx, dx, pnt, vec)
    _assert_eq(geomid, 3, 'geom_id')
    pnt, vec, unused = np.array(pnt), np.array(vec), np.zeros(1, dtype=np.int32)
    mj_dist = mujoco.mj_ray(m, d, pnt, vec, None, 1, -1, unused)
    _assert_eq(dist, mj_dist, 'dist')

    # looking up at box from below
    pnt, vec = jp.array([1, 0, 0.05]), jp.array([0, 0.05, 1.0])
    vec /= jp.linalg.norm(vec)
    dist, geomid = jax.jit(mjx.ray)(mx, dx, pnt, vec)
    _assert_eq(geomid, 3, 'geom_id')
    pnt, vec, unused = np.array(pnt), np.array(vec), np.zeros(1, dtype=np.int32)
    mj_dist = mujoco.mj_ray(m, d, pnt, vec, None, 1, -1, unused)
    _assert_eq(dist, mj_dist, 'dist')

  def test_ray_mesh(self):
    """Tests MJX ray<>mesh matches MuJoCo."""
    m = test_util.load_test_file('ray.xml')
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    mx, dx = mjx.put_model(m), mjx.put_data(m, d)

    # look at the tetrahedron
    pnt, vec = jp.array([2.0, 2.0, 2.0]), -jp.array([1.0, 1.0, 1.0])
    vec /= jp.linalg.norm(vec)
    dist, geomid = jax.jit(mjx.ray)(mx, dx, pnt, vec)
    _assert_eq(geomid, 4, 'geom_id')

    pnt, vec, geomid = np.array(pnt), np.array(vec), np.zeros(1, dtype=np.int32)
    mj_dist = mujoco.mj_ray(m, d, pnt, vec, None, 1, -1, geomid)
    _assert_eq(geomid, 4, 'geom_id')
    _assert_eq(dist, mj_dist, 'dist-tetrahedron')

    # look away from the dodecahedron
    pnt, vec = jp.array([4.0, 2.0, 2.0]), jp.array([2.0, 1.0, 1.0])
    vec /= jp.linalg.norm(vec)
    _, geomid = jax.jit(mjx.ray)(mx, dx, pnt, vec)
    _assert_eq(geomid, -1, 'geom_id')

    # look at the dodecahedron
    pnt, vec = jp.array([4.0, 2.0, 2.0]), -jp.array([2.0, 1.0, 1.0])
    vec /= jp.linalg.norm(vec)
    dist, geomid = jax.jit(mjx.ray)(mx, dx, pnt, vec)
    _assert_eq(geomid, 5, 'geom_id')

    pnt, vec, geomid = np.array(pnt), np.array(vec), np.zeros(1, dtype=np.int32)
    mj_dist = mujoco.mj_ray(m, d, pnt, vec, None, 1, -1, geomid)
    _assert_eq(geomid, 5, 'geom_id')
    _assert_eq(dist, mj_dist, 'dist-dodecahedron')

  def test_ray_geomgroup(self):
    """Tests ray geomgroup filter."""
    m = test_util.load_test_file('ray.xml')
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    mx, dx = mjx.put_model(m), mjx.put_data(m, d)
    ray_fn = jax.jit(mjx.ray, static_argnums=(4,))

    # hits plane with geom_group[0] = 1
    pnt, vec = jp.array([2, 1, 3.0]), jp.array([0.1, 0.2, -1.0])
    vec /= jp.linalg.norm(vec)
    geomgroup = (1, 0, 0, 0, 0, 0)
    dist, geomid = ray_fn(mx, dx, pnt, vec, geomgroup)
    _assert_eq(geomid, 0, 'geom_id')
    pnt, vec, unused = np.array(pnt), np.array(vec), np.zeros(1, dtype=np.int32)
    mj_dist = mujoco.mj_ray(m, d, pnt, vec, None, 1, -1, unused)
    _assert_eq(dist, mj_dist, 'dist')

    # nothing hit with geom_group[0] = 0
    pnt, vec = jp.array([2, 1, 3.0]), jp.array([0.1, 0.2, -1.0])
    vec /= jp.linalg.norm(vec)
    geomgroup = (0, 0, 0, 0, 0, 0)
    dist, geomid = ray_fn(mx, dx, pnt, vec, geomgroup)
    _assert_eq(geomid, -1, 'geom_id')
    _assert_eq(dist, -1, 'dist')

  def test_ray_flg_static(self):
    """Tests ray flg_static filter."""
    m = test_util.load_test_file('ray.xml')
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    mx, dx = mjx.put_model(m), mjx.put_data(m, d)
    ray_fn = jax.jit(mjx.ray, static_argnames=('flg_static',))

    # nothing hit with flg_static = False
    pnt, vec = jp.array([2, 1, 3.0]), jp.array([0.1, 0.2, -1.0])
    vec /= jp.linalg.norm(vec)
    dist, geomid = ray_fn(mx, dx, pnt, vec, flg_static=False)
    _assert_eq(geomid, -1, 'geom_id')
    _assert_eq(dist, -1, 'dist')

  @parameterized.named_parameters(
      ('int', 0),
      ('sequence', (0,)),
  )
  def test_ray_bodyexclude(self, bodyexclude):
    """Tests ray bodyexclude filter."""
    m = test_util.load_test_file('ray.xml')
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    mx, dx = mjx.put_model(m), mjx.put_data(m, d)
    ray_fn = jax.jit(mjx.ray, static_argnames=('bodyexclude',))

    # The ray should hit the plane (geom 0, body 0), but body 0 is excluded.
    pnt, vec = jp.array([2, 1, 3.0]), jp.array([0.1, 0.2, -1.0])
    vec /= jp.linalg.norm(vec)
    dist, geomid = ray_fn(mx, dx, pnt, vec, bodyexclude=bodyexclude)
    _assert_eq(geomid, -1, 'geom_id')
    _assert_eq(dist, -1, 'dist')

  def test_ray_invisible(self):
    """Tests ray doesn't hit transparent geoms."""
    m = test_util.load_test_file('ray.xml')
    # nothing hit with transparent geoms:
    m.geom_rgba = 0
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    mx, dx = mjx.put_model(m), mjx.put_data(m, d)

    pnt, vec = jp.array([2, 1, 3.0]), jp.array([0.1, 0.2, -1.0])
    vec /= jp.linalg.norm(vec)
    dist, geomid = jax.jit(mjx.ray)(mx, dx, pnt, vec)
    _assert_eq(geomid, -1, 'geom_id')
    _assert_eq(dist, -1, 'dist')

  def test_ray_hfield(self):
    """Tests that MJX ray<>hfield matches MuJoCo."""
    m = test_util.load_test_file('ray.xml')
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    mx, dx = mjx.put_model(m), mjx.put_data(m, d)
    ray_fn = jax.jit(mjx.ray, static_argnums=(4,))
    # Find hfield geom ID to test only hfield intersections
    hfield_geom_id = -1
    for i in range(m.geom_type.shape[0]):
      if m.geom_type[i] == mujoco.mjx._src.types.GeomType.HFIELD:
        hfield_geom_id = i
        break
    
    # Set all groups to False except the one containing hfield
    geomgroup = np.zeros(mujoco.mjNGROUP, dtype=bool)
    if hfield_geom_id >= 0:
      hfield_group = m.geom_group[hfield_geom_id]
      geomgroup[hfield_group] = True
    geomgroup = tuple(geomgroup.tolist())

    # Test 1: Single ray hitting hfield from directly above
    # Hfield is at [20, 20, 20] with size [.6, .4, .1, .1]
    pnt, vec = jp.array([20.0, 20.0, 25.0]), jp.array([0.0, 0.0, -1.0])
    dist, geomid = ray_fn(mx, dx, pnt, vec, geomgroup)
    
    mj_geomid = np.zeros(1, dtype=np.int32)
    mj_dist = mujoco.mj_ray(m, d, pnt, vec, geomgroup, True, -1, mj_geomid)
    
    assert mj_dist != -1, 'MuJoCo ray should hit hfield, test case setup might be wrong.'
    _assert_eq(dist, mj_dist, 'hfield_single_dist')
    _assert_eq(geomid, mj_geomid[0], 'hfield_single_geomid')

    # Test 2: Sample random ray origin
    # Hfield is at [20, 20, 20] with size [.6, .4, .1, .1]
    x_positions = jp.linspace(19.5, 20.5, 5)  # Within hfield x bounds
    y_positions = jp.linspace(19.7, 20.3, 5)  # Within hfield y bounds
    z_start = 25.0  # Well above hfield
    
    ray_directions = [
        jp.array([0.0, 0.0, -1.0]),  # Straight down
        jp.array([0.1, 0.0, -1.0]),  # Slight angle in x
        jp.array([0.0, 0.1, -1.0]),  # Slight angle in y
        jp.array([0.1, 0.1, -1.0]),  # Diagonal angle
    ]
    
    test_count = 0
    for x in x_positions:
      for y in y_positions:
        for vec_unnorm in ray_directions:
          vec = vec_unnorm / jp.linalg.norm(vec_unnorm)  # Normalize
          pnt = jp.array([x, y, z_start])
          
          # MJX ray (hfield only)
          dist_mjx, geomid_mjx = ray_fn(mx, dx, pnt, vec, geomgroup)
          
          # MuJoCo ground truth (hfield only)
          mj_geomid = np.zeros(1, dtype=np.int32)
          mj_dist = mujoco.mj_ray(m, d, pnt, vec, geomgroup, True, -1, mj_geomid)
          
          # Assert equality
          _assert_eq(dist_mjx, mj_dist, f'grid_dist_{test_count}')
          _assert_eq(geomid_mjx, mj_geomid[0], f'grid_geomid_{test_count}')
          test_count += 1

    # Test 3: Rays that should miss hfield (outside bounds)
    miss_tests = [
        (jp.array([25.0, 20.0, 25.0]), jp.array([0.0, 0.0, -1.0])),  # Outside x bounds
        (jp.array([20.0, 25.0, 25.0]), jp.array([0.0, 0.0, -1.0])),  # Outside y bounds  
        (jp.array([20.0, 20.0, 25.0]), jp.array([1.0, 0.0, 0.0])),   # Horizontal ray
    ]
    
    for i, (pnt_miss, vec_miss) in enumerate(miss_tests):
      dist_miss, geomid_miss = ray_fn(mx, dx, pnt_miss, vec_miss, geomgroup)
      
      mj_geomid_miss = np.zeros(1, dtype=np.int32)
      mj_dist_miss = mujoco.mj_ray(m, d, pnt_miss, vec_miss, geomgroup, True, -1, mj_geomid_miss)
      
      _assert_eq(dist_miss, mj_dist_miss, f'miss_dist_{i}')
      _assert_eq(dist_miss, -1, f'miss_dist_{i}_check')
      _assert_eq(geomid_miss, mj_geomid_miss[0], f'miss_geomid_{i}')

    # Test 4: Angular rays from different positions
    center_pos = jp.array([20.0, 20.0, 22.0])  # Above hfield center
    
    # Random angles
    angles = jp.linspace(0, jp.pi/4, 5)  # 0 to 45 degrees from vertical
    for i, angle in enumerate(angles):
      # Create angled ray (rotating around x-axis)
      vec_angled = jp.array([0.0, jp.sin(angle), -jp.cos(angle)])
      
      dist_angled, geomid_angled = ray_fn(mx, dx, center_pos, vec_angled, geomgroup)
      
      mj_geomid_angled = np.zeros(1, dtype=np.int32)
      mj_dist_angled = mujoco.mj_ray(m, d, center_pos, vec_angled, geomgroup, True, -1, mj_geomid_angled)
      
      _assert_eq(dist_angled, mj_dist_angled, f'angle_dist_{i}')
      _assert_eq(geomid_angled, mj_geomid_angled[0], f'angle_geomid_{i}')


if __name__ == '__main__':
  absltest.main()
