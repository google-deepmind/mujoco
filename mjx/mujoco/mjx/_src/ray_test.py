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

  def test_ray_hfield(self):
    """Tests MJX ray<>hfield matches MuJoCo.

    Before HFIELD was added to _RAY_FUNC, the dispatch loop silently skipped
    height field geoms: every one of the surface/side/base rays below returned
    (dist=-1, geomid=-1), i.e. terrain was invisible to mjx.ray while the same
    rays hit with mujoco.mj_ray.
    """
    xml = """
    <mujoco>
      <asset>
        <hfield name="terrain" nrow="9" ncol="11" size="1.0 0.8 0.5 0.1"/>
        <material name="mat0" rgba="1 1 1 1"/>
      </asset>
      <worldbody>
        <geom name="hf" type="hfield" hfield="terrain" pos="0.05 -0.02 0.03"/>
        <geom name="ball" type="sphere" size="0.1" pos="0.3 0.2 1.2"
              material="mat0"/>
      </worldbody>
    </mujoco>
    """
    m = mujoco.MjModel.from_xml_string(xml)
    # deterministic bumpy terrain, normalized heights in [0, 1]
    rr, cc = np.meshgrid(np.arange(9), np.arange(11), indexing='ij')
    m.hfield_data[:] = (0.5 + 0.5 * np.sin(0.9 * rr) * np.cos(0.7 * cc)).ravel()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    mx, dx = mjx.put_model(m), mjx.put_data(m, d)

    rays = [
        # straight down onto the terrain surface
        ([0.12, -0.23, 2.0], [0.0, 0.0, -1.0]),
        ([-0.5, 0.4, 2.0], [0.0, 0.0, -1.0]),
        # oblique onto the surface
        ([1.5, 0.8, 1.5], [-0.6, -0.35, -1.0]),
        # from the side, hitting a prism side face below the terrain edge
        ([2.0, 0.1, 0.1], [-1.0, 0.0, 0.0]),
        ([0.0, -2.0, 0.2], [0.05, 1.0, 0.0]),
        # from below, hitting the bottom of the base box
        ([0.0, 0.0, -1.0], [0.0, 0.0, 1.0]),
        # hits the sphere above the terrain
        ([0.3, 0.2, 2.5], [0.0, 0.0, -1.0]),
        # misses
        ([3.0, 3.0, 2.0], [0.0, 0.0, -1.0]),
        ([-2.0, 0.0, 0.55], [1.0, 0.0, 0.0]),
    ]
    ray_fn = jax.jit(mjx.ray)
    for pnt, vec in rays:
      pnt, vec = np.array(pnt), np.array(vec)
      vec /= np.linalg.norm(vec)
      geomid_mj = np.zeros(1, dtype=np.int32)
      dist_mj = mujoco.mj_ray(m, d, pnt, vec, None, 1, -1, geomid_mj)
      dist, geomid = ray_fn(mx, dx, jp.array(pnt), jp.array(vec))
      _assert_eq(dist, dist_mj, f'dist pnt={pnt} vec={vec}')
      _assert_eq(geomid, geomid_mj[0], f'geom_id pnt={pnt} vec={vec}')

  def test_ray_unsupported_geom_warning(self):
    """Tests that ray warns about geom types it cannot intersect."""
    xml = """
    <mujoco>
      <asset>
        <material name="mat0" rgba="1 1 1 1"/>
      </asset>
      <worldbody>
        <geom name="cyl" type="cylinder" size="0.1 0.3" pos="0 0 1"
              material="mat0"/>
        <geom name="ball" type="sphere" size="0.1" pos="1 0 1" material="mat0"/>
      </worldbody>
    </mujoco>
    """
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    mx, dx = mjx.put_model(m), mjx.put_data(m, d)

    pnt, vec = jp.array([0, 0, 3.0]), jp.array([0, 0, -1.0])
    with self.assertWarnsRegex(UserWarning, 'CYLINDER'):
      dist, geomid = mjx.ray(mx, dx, pnt, vec)
    # the cylinder is invisible to the ray: it reports a miss
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


if __name__ == '__main__':
  absltest.main()
