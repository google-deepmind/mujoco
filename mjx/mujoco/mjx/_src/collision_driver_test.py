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
"""Tests the collision driver."""

import dataclasses
from typing import Dict, Optional, Tuple

from absl.testing import absltest
from absl.testing import parameterized
from etils import epath
import jax
import jax.numpy as jp
import mujoco
from mujoco import mjx
from mujoco.mjx._src import collision_driver
from mujoco.mjx._src import test_util
# pylint: disable=g-importing-member
from mujoco.mjx._src.types import Contact
from mujoco.mjx._src.types import Data
from mujoco.mjx._src.types import DisableBit
from mujoco.mjx._src.types import Model
# pylint: enable=g-importing-member
import numpy as np


def _assert_attr_eq(mjx_d, mj_d, attr, name, atol):
  if attr == 'efc_address':
    # we do not test efc_address since it gets set in constraint logic
    return
  err_msg = f'mismatch: {attr} in run: {name}'
  mjx_d, mj_d = getattr(mjx_d, attr), getattr(mj_d, attr)
  if attr == 'frame':
    mj_d = mj_d.reshape((-1, 3, 3))
  if mjx_d.shape != mj_d.shape:
    raise AssertionError(f'{attr} shape mismatch: {mjx_d.shape}, {mj_d.shape}')
  np.testing.assert_allclose(mjx_d, mj_d, err_msg=err_msg, atol=atol)


def _collide(
    mjcf: str, assets: Optional[Dict[str, str]] = None
) -> Tuple[mujoco.MjModel, mujoco.MjData, Model, Data]:
  m = mujoco.MjModel.from_xml_string(mjcf, assets or {})
  mx = mjx.put_model(m)
  d = mujoco.MjData(m)
  dx = mjx.put_data(m, d)

  mujoco.mj_step(m, d)
  collision_jit_fn = jax.jit(mjx.collision)
  kinematics_jit_fn = jax.jit(mjx.kinematics)
  dx = kinematics_jit_fn(mx, dx)
  dx = collision_jit_fn(mx, dx)
  return d, dx


class SphereCollisionTest(parameterized.TestCase):
  _SPHERE_PLANE = """
    <mujoco>
      <worldbody>
        <geom size="40 40 40" type="plane"/>
        <body pos="0 0 0.4">
          <joint type="free"/>
          <geom size="0.5" type="sphere"/>
        </body>
      </worldbody>
    </mujoco>
  """

  _SPHERE_SPHERE = """
    <mujoco>
      <worldbody>
        <body>
          <joint type="free"/>
          <geom pos="0 0 0" size="0.2" type="sphere"/>
        </body>
        <body >
          <joint type="free"/>
          <geom pos="0 0.3 0" size="0.11" type="sphere"/>
        </body>
      </worldbody>
    </mujoco>
  """

  _SPHERE_CAP = """
    <mujoco>
      <worldbody>
        <body>
          <joint type="free"/>
          <geom pos="0 0.3 0" size="0.05" type="sphere"/>
        </body>
        <body>
          <joint axis="1 0 0" type="free"/>
          <geom fromto="0.0 -0.5 0.14 0.0 0.5 0.14" size="0.1" type="capsule"/>
        </body>
      </worldbody>
    </mujoco>
  """

  @parameterized.parameters(
      ('sphere_plane', _SPHERE_PLANE),
      ('sphere_sphere', _SPHERE_SPHERE),
      ('sphere_cap', _SPHERE_CAP),
  )
  def test_sphere(self, name, mjcf):
    d, dx = _collide(mjcf)
    for field in dataclasses.fields(Contact):
      _assert_attr_eq(dx.contact, d.contact, field.name, name, 1e-5)

  _SPHERE_CONVEX = """
    <mujoco>
      <worldbody>
        <body pos="0.52 0 0.52">
          <joint axis="1 0 0" type="free"/>
          <geom size="0.05" type="sphere"/>
        </body>
        <body>
          <joint axis="1 0 0" type="free"/>
          <geom size="0.5 0.5 0.5" type="box"/>
        </body>
      </worldbody>
    </mujoco>
  """

  def test_sphere_convex(self):
    d, dx = _collide(self._SPHERE_CONVEX)

    for field in dataclasses.fields(Contact):
      _assert_attr_eq(dx.contact, d.contact, field.name, 'sphere_convex', 1e-4)


class CapsuleCollisionTest(parameterized.TestCase):
  _CAP_PLANE = """
    <mujoco>
      <worldbody>
        <geom size="40 40 40" type="plane"/>
        <body pos="0 0 0.4">
          <joint type="free"/>
          <geom fromto="-1 0 0 1 0 0" size="0.5" type="capsule"/>
        </body>
      </worldbody>
    </mujoco>
  """

  _CAP_CAP = """
    <mujoco model="two_capsules">
      <worldbody>
        <body>
          <joint type="free"/>
          <geom fromto="0.62235904  0.58846647 0.651046 1.5330081 0.33564585 0.977849"
           size="0.05" type="capsule"/>
        </body>
        <body>
          <joint type="free"/>
          <geom fromto="0.5505271 0.60345304 0.476661 1.3900293 0.30709633 0.932082"
           size="0.05" type="capsule"/>
        </body>
      </worldbody>
    </mujoco>
  """

  @parameterized.parameters(
      ('capsule_plane', _CAP_PLANE),
      ('capsule_capsule', _CAP_CAP),
  )
  def test_capsule(self, name, mjcf):
    d, dx = _collide(mjcf)
    for field in dataclasses.fields(Contact):
      _assert_attr_eq(dx.contact, d.contact, field.name, name, 1e-4)

  _PARALLEL_CAP = """
    <mujoco>
      <worldbody>
        <body>
          <joint type="free"/>
          <geom fromto="-0.5 0.1 0.25 0.5 0.1 0.25" size="0.1" type="capsule"/>
        </body>
        <body>
          <joint type="free"/>
          <geom fromto="-0.5 0.1 0.1 0.5 0.1 0.1" size="0.1" type="capsule"/>
        </body>
      </worldbody>
    </mujoco>
  """

  def test_parallel_capsules(self):
    """Tests that two parallel capsules are colliding at the midpoint."""
    _, dx = _collide(self._PARALLEL_CAP)

    np.testing.assert_allclose(dx.contact.dist, -0.05)
    np.testing.assert_allclose(
        dx.contact.pos[0],
        np.array([0.0, 0.1, (0.15 + 0.2) / 2.0]),
        atol=1e-5,
    )
    np.testing.assert_allclose(
        dx.contact.frame[0, 0, :], np.array([0, 0.0, -1.0]), atol=1e-5
    )

  _CAP_BOX = """
    <mujoco>
      <worldbody>
        <body pos="0 0 0.54">
          <joint axis="1 0 0" type="free"/>
          <geom fromto="-0.4 0 0 0.4 0 0" size="0.05" type="capsule"/>
        </body>
        <body>
          <joint axis="1 0 0" type="free"/>
          <geom size="0.5 0.5 0.5" type="box"/>
        </body>
      </worldbody>
    </mujoco>
  """

  def test_capsule_convex(self):
    """Tests a capsule-convex collision for a face contact."""
    d, dx = _collide(self._CAP_BOX)

    for field in dataclasses.fields(Contact):
      _assert_attr_eq(dx.contact, d.contact, field.name, 'capsule_convex', 1e-4)

  _CAP_EDGE_BOX = """
    <mujoco>
      <worldbody>
        <body pos="0.5 0 0.55" euler="0 30 0">
          <joint axis="1 0 0" type="free"/>
          <geom fromto="-0.6 0 0 0.6 0 0" size="0.05" type="capsule"/>
        </body>
        <body>
          <joint axis="1 0 0" type="free"/>
          <geom size="0.5 0.5 0.5" type="box"/>
        </body>
      </worldbody>
    </mujoco>
  """

  def test_capsule_convex_edge(self):
    """Tests a capsule-convex collision for an edge contact."""
    d, dx = _collide(self._CAP_EDGE_BOX)

    c = dx.contact
    self.assertEqual(c.pos.shape[0], 2)
    self.assertGreater(c.dist[1], 0)
    # extract the contact point with penetration
    c = jax.tree_map(lambda x: jp.take(x, 0, axis=0)[None], dx.contact)
    for field in dataclasses.fields(Contact):
      _assert_attr_eq(c, d.contact, field.name, 'capsule_convex_edge', 1e-4)


class ConvexTest(absltest.TestCase):
  """Tests the convex contact functions."""

  _BOX_PLANE = """
    <mujoco>
      <worldbody>
        <geom size="40 40 40" type="plane"/>
        <body pos="0 0 0.7" euler="45 0 0">
          <joint axis="1 0 0" type="free"/>
          <geom size="0.5 0.5 0.5" type="box"/>
        </body>
      </worldbody>
    </mujoco>
  """

  def test_box_plane(self):
    """Tests box collision with a plane."""
    d, dx = _collide(self._BOX_PLANE)

    np.testing.assert_array_less(dx.contact.dist[:2], 0)
    np.testing.assert_array_less(-dx.contact.dist[2:], 0)
    # extract the contact points with penetration
    c = jax.tree_map(lambda x: jp.take(x, jp.array([0, 1]), axis=0), dx.contact)
    for field in dataclasses.fields(Contact):
      _assert_attr_eq(c, d.contact, field.name, 'box_plane', 1e-2)

  _BOX_BOX = """
    <mujoco>
      <worldbody>
        <body pos="0.0 1.0 0.2">
          <joint axis="1 0 0" type="free"/>
          <geom size="0.2 0.2 0.2" type="box"/>
        </body>
        <body pos="0.1 1.0 0.495" euler="0.1 -0.1 0">
          <joint axis="1 0 0" type="free"/>
          <geom size="0.1 0.1 0.1" type="box"/>
        </body>
      </worldbody>
    </mujoco>
  """

  def test_box_box(self):
    """Tests a face contact for a box-box collision."""
    d, dx = _collide(self._BOX_BOX)
    c = dx.contact

    self.assertEqual(c.pos.shape[0], 4)
    np.testing.assert_array_less(c.dist, 0)
    np.testing.assert_array_almost_equal(c.pos[:, 2], np.array([0.39] * 4), 2)
    np.testing.assert_array_almost_equal(
        c.frame[:, 0, :], np.array([[0.0, 0.0, 1.0]] * 4)
    )
    np.testing.assert_array_almost_equal(
        c.frame.reshape((-1, 9)), d.contact.frame[:4, :]
    )

  _BOX_BOX_EDGE = """
    <mujoco>
      <worldbody>
        <body pos="-1.0 -1.0 0.2">
          <joint axis="1 0 0" type="free"/>
          <geom size="0.2 0.2 0.2" type="box"/>
        </body>
        <body pos="-1.0 -1.2 0.55" euler="0 45 30">
          <joint axis="1 0 0" type="free"/>
          <geom size="0.1 0.1 0.1" type="box"/>
        </body>
      </worldbody>
    </mujoco>
  """

  def test_box_box_edge(self):
    """Tests an edge contact for a box-box collision."""
    d, dx = _collide(self._BOX_BOX_EDGE)

    # Only one contact point.
    np.testing.assert_array_less(dx.contact.dist[:1], 0)
    np.testing.assert_array_less(-dx.contact.dist[1:], 0)
    # extract the contact point with penetration
    c = jax.tree_map(lambda x: jp.take(x, 0, axis=0)[None], dx.contact)
    for field in dataclasses.fields(Contact):
      _assert_attr_eq(c, d.contact, field.name, 'box_box_edge', 1e-2)

  _CONVEX_CONVEX = """
    <mujoco>
      <asset>
        <mesh name="tetrahedron" file="meshes/tetrahedron.stl" scale="0.1 0.1 0.1" />
        <mesh name="dodecahedron" file="meshes/dodecahedron.stl" scale="0.01 0.01 0.01" />
      </asset>
      <worldbody>
        <body pos="0.0 2.0 0.096">
          <joint axis="1 0 0" type="free"/>
          <geom size="0.2 0.2 0.2" type="mesh" mesh="tetrahedron"/>
        </body>
        <body pos="0.0 2.0 0.289" euler="0.1 -0.1 45">
          <joint axis="1 0 0" type="free"/>
          <geom size="0.1 0.1 0.1" type="mesh" mesh="dodecahedron"/>
        </body>
      </worldbody>
    </mujoco>
  """

  def test_convex_convex(self):
    """Tests generic convex-convex collision."""
    directory = epath.resource_path('mujoco.mjx')
    assets = {
        'meshes/tetrahedron.stl': (
            directory / 'test_data' / 'meshes/tetrahedron.stl'
        ).read_bytes(),
        'meshes/dodecahedron.stl': (
            directory / 'test_data' / 'meshes/dodecahedron.stl'
        ).read_bytes(),
    }
    _, dx = _collide(self._CONVEX_CONVEX, assets=assets)
    c = dx.contact

    # Only one contact point for an edge contact.
    self.assertLess(c.dist[0], 0)
    np.testing.assert_array_less(0, c.dist[1:])
    np.testing.assert_array_almost_equal(c.frame[0, 0], np.array([0, 0, 1]))


class BodyPairFilterTest(absltest.TestCase):
  """Tests that certain body pairs get filtered."""

  _SELF_COLLISION = """
    <mujoco>
      <worldbody>
        <body>
          <joint type="free"/>
          <geom size="0.2"/>
          <geom size="0.2"/>
        </body>
      </worldbody>
    </mujoco>
  """

  def test_filter_self_collision(self):
    """Tests that self collisions get filtered."""
    d, dx = _collide(self._SELF_COLLISION)
    self.assertEqual(dx.contact.pos.shape[0], d.contact.pos.shape[0])
    self.assertEqual(dx.contact.pos.shape[0], 0)

  _PARENT_CHILD = """
    <mujoco>
      <worldbody>
        <body>
          <joint type="free"/>
          <geom size="0.2"/>
          <body pos="0.0 0.0 0.1">
            <joint type="hinge"/>
            <geom size="0.2"/>
          </body>
        </body>
      </worldbody>
    </mujoco>
  """

  def test_filter_parent_child(self):
    """Tests that parent-child collisions get filtered."""
    m = mujoco.MjModel.from_xml_string(self._PARENT_CHILD)
    mx = mjx.put_model(m)
    d = mujoco.MjData(m)
    dx = mjx.put_data(m, d)

    mujoco.mj_step(m, d)
    collision_jit_fn = jax.jit(mjx.collision)
    kinematics_jit_fn = jax.jit(mjx.kinematics)
    dx = kinematics_jit_fn(mx, dx)
    dx = collision_jit_fn(mx, dx)

    self.assertEqual(dx.contact.pos.shape[0], d.contact.pos.shape[0])
    self.assertEqual(dx.contact.pos.shape[0], 0)

  def test_disable_filter_parent_child(self):
    """Tests that filterparent flag disables parent-child filtering."""
    m = mujoco.MjModel.from_xml_string(self._PARENT_CHILD)
    m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_FILTERPARENT
    mx = mjx.put_model(m)
    d = mujoco.MjData(m)
    dx = mjx.put_data(m, d)

    mujoco.mj_step(m, d)
    collision_jit_fn = jax.jit(mjx.collision)
    kinematics_jit_fn = jax.jit(mjx.kinematics)
    dx = kinematics_jit_fn(mx, dx)
    dx = collision_jit_fn(mx, dx)

    # one collision between parent-child spheres
    self.assertEqual(dx.contact.pos.shape[0], d.contact.pos.shape[0])
    self.assertEqual(dx.contact.pos.shape[0], 1)


class NconTest(parameterized.TestCase):
  """Tests ncon."""

  def test_ncon(self):
    m = test_util.load_test_file('constraints.xml')
    ncon = collision_driver.ncon(m)
    self.assertEqual(ncon, 16)

  def test_disable_contact(self):
    m = test_util.load_test_file('constraints.xml')
    m.opt.disableflags |= DisableBit.CONTACT
    ncon = collision_driver.ncon(m)
    self.assertEqual(ncon, 0)


class TopKContactTest(absltest.TestCase):
  """Tests top-k contacts."""

  _CAPSULES = """
    <mujoco>
      <custom>
        <numeric data="2" name="max_contact_points"/>
      </custom>
      <worldbody>
        <body pos="0 0 0.54">
          <joint axis="1 0 0" type="free"/>
          <geom fromto="-0.4 0 0 0.4 0 0" size="0.05" type="capsule"/>
        </body>
        <body pos="0 0 0.54">
          <joint axis="1 0 0" type="free"/>
          <geom fromto="-0.4 0 0 0.4 0 0" size="0.05" type="capsule"/>
        </body>
        <body pos="0 0 0.54">
          <joint axis="1 0 0" type="free"/>
          <geom fromto="-0.4 0 0 0.4 0 0" size="0.05" type="capsule"/>
        </body>
      </worldbody>
    </mujoco>
  """

  def test_top_k_contacts(self):
    m = mujoco.MjModel.from_xml_string(self._CAPSULES)
    mx_top_k = mjx.put_model(m)
    mx_all = mx_top_k.replace(
        nnumeric=0, name_numericadr=np.array([]), numeric_data=np.array([])
    )
    d = mujoco.MjData(m)
    dx = mjx.put_data(m, d)

    collision_jit_fn = jax.jit(mjx.collision)
    kinematics_jit_fn = jax.jit(mjx.kinematics)
    dx = kinematics_jit_fn(mx_all, dx)

    dx_all = collision_jit_fn(mx_all, dx)
    dx_top_k = collision_jit_fn(mx_top_k, dx)

    self.assertEqual(dx_all.contact.dist.shape, (3,))
    self.assertEqual(dx_top_k.contact.dist.shape, (2,))


if __name__ == '__main__':
  absltest.main()
