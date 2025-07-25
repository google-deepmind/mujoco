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

  def test_sphere_convex_face(self):
    # no contact
    xml = self._SPHERE_CONVEX.replace(
        '<body pos="0.52 0 0.52">', '<body pos="0.55 0 0.5">'
    )
    d, dx = _collide(xml)
    self.assertEmpty(d.contact.dist)
    self.assertGreater(dx.contact.dist, 0)

    # face contact
    xml = self._SPHERE_CONVEX.replace(
        '<body pos="0.52 0 0.52">', '<body pos="0.51 0 0.25">'
    )
    d, dx = _collide(xml)
    for field in dataclasses.fields(Contact):
      _assert_attr_eq(dx.contact, d.contact, field.name, 'face', 1e-4)

    # deep face contact
    xml = self._SPHERE_CONVEX.replace(
        '<body pos="0.52 0 0.52">', '<body pos="0.48 0 0.47">'
    )
    d, dx = _collide(xml)
    self.assertTrue((dx.contact.dist < 0).all())
    self.assertTrue((d.contact.dist < 0).all())
    np.testing.assert_allclose(dx.contact.dist, [-0.07], atol=1e-5)
    np.testing.assert_array_almost_equal(dx.contact.pos, d.contact.pos)
    np.testing.assert_array_almost_equal(
        dx.contact.frame, d.contact.frame.reshape((-1, 3, 3))
    )

  def test_sphere_convex_edge(self):
    # edge contact
    d, dx = _collide(self._SPHERE_CONVEX)
    for field in dataclasses.fields(Contact):
      _assert_attr_eq(dx.contact, d.contact, field.name, 'edge', 1e-4)

    # deep edge penetration
    xml = self._SPHERE_CONVEX.replace(
        '<body pos="0.52 0 0.52">', '<body pos="0.49 0 0.49">'
    )
    d, dx = _collide(xml)
    self.assertTrue((dx.contact.dist < 0).all())
    self.assertTrue((d.contact.dist < 0).all())
    np.testing.assert_allclose(dx.contact.dist, [-0.06], atol=1e-5)
    np.testing.assert_array_almost_equal(dx.contact.pos, d.contact.pos)
    np.testing.assert_array_almost_equal(
        dx.contact.frame, d.contact.frame.reshape((-1, 3, 3))
    )

    # vertex contact
    xml = self._SPHERE_CONVEX.replace(
        '<body pos="0.52 0 0.52">', '<body pos="0.5 0.52 0.51">'
    )
    d, dx = _collide(xml)
    for field in dataclasses.fields(Contact):
      _assert_attr_eq(dx.contact, d.contact, field.name, 'vertex', 1e-4)

    # sphere center on vertex
    xml = self._SPHERE_CONVEX.replace(
        '<body pos="0.52 0 0.52">', '<body pos="0.5 0 0.5">'
    )
    d, dx = _collide(xml)
    for field in dataclasses.fields(Contact):
      _assert_attr_eq(dx.contact, d.contact, field.name, 'vertex_center', 1e-4)


class EllipsoidCollisionTest(parameterized.TestCase):

  _ELLIPSOID_PLANE = """
    <mujoco>
      <worldbody>
        <geom name="floor" size="0 0 .05" type="plane"/>
        <body pos="0 0 0.03" euler="45 0 0">
          <freejoint/>
          <geom size=".15 .03 .05" type="ellipsoid"/>
        </body>
      </worldbody>
    </mujoco>
  """

  def test_plane_ellipsoid(self):
    """Tests ellipsoid plane contact."""
    d, dx = _collide(self._ELLIPSOID_PLANE)
    self.assertLess(dx.contact.dist[0], 0)
    for field in dataclasses.fields(Contact):
      _assert_attr_eq(
          dx.contact, d.contact, field.name, 'ellipsoid-plane', 1e-5)

  _ELLIPSOID_ELLIPSOID = """
    <mujoco>
      <worldbody>
        <body>
          <geom size=".15 .03 .05" type="ellipsoid"/>
        </body>
        <body pos="0 0 0.09">
          <freejoint/>
          <geom size=".15 .03 .05" type="ellipsoid"/>
        </body>
      </worldbody>
    </mujoco>
  """

  def test_ellipsoid_ellipsoid(self):
    """Tests ellipsoid ellipsoid contact."""
    d, dx = _collide(self._ELLIPSOID_ELLIPSOID)
    self.assertLess(dx.contact.dist[0], 0)
    for field in dataclasses.fields(Contact):
      _assert_attr_eq(
          dx.contact, d.contact, field.name, 'ellipsoid-ellipsoid', 1e-5)

  _ELLIPSOID_CAPSULE = """
    <mujoco>
      <worldbody>
        <body>
          <geom size=".15 .03 .05" type="ellipsoid"/>
        </body>
        <body pos="0 0 0.0999">
          <freejoint/>
          <geom size=".05" fromto="-.1 0 0 .1 0 0" type="capsule"/>
        </body>
      </worldbody>
    </mujoco>
  """

  def test_capsule_ellipsoid(self):
    """Tests ellipsoid capsule contact."""
    d, dx = _collide(self._ELLIPSOID_CAPSULE)
    self.assertLess(dx.contact.dist[0], 0)
    for field in dataclasses.fields(Contact):
      _assert_attr_eq(
          dx.contact, d.contact, field.name, 'ellipsoid-capsule', 1e-4)

  _ELLIPSOID_CYLINDER = """
    <mujoco>
      <worldbody>
        <body>
          <geom size=".15 .05" type="cylinder"/>
        </body>
        <body pos="0 0 0.09">
          <freejoint/>
          <geom size=".15 .03 .05" type="ellipsoid"/>
        </body>
      </worldbody>
    </mujoco>
  """

  def test_ellipsoid_cylinder(self):
    """Tests ellipsoid cylinder contact."""
    d, dx = _collide(self._ELLIPSOID_CYLINDER)
    d.contact.pos[0][2] = 0.04  # MJX finds the deepest point on the surface
    self.assertLess(dx.contact.dist[0], 0)
    for field in dataclasses.fields(Contact):
      _assert_attr_eq(
          dx.contact, d.contact, field.name, 'ellipsoid-cylinder', 1e-4)


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
          <geom fromto="-0.4 0 0 1.0 0 0" size="0.05" type="capsule"/>
        </body>
        <body>
          <joint axis="1 0 0" type="free"/>
          <geom size="0.5 0.5 0.5" type="box"/>
        </body>
      </worldbody>
    </mujoco>
  """

  def test_capsule_convex_face(self):
    """Tests face contact."""
    d, dx = _collide(self._CAP_BOX)

    # sort positions for comparison
    idx = np.lexsort((dx.contact.pos[:, 0], dx.contact.pos[:, 1]))
    dx = dx.tree_replace({'contact.pos': dx.contact.pos[idx]})
    idx = np.lexsort((d.contact.pos[:, 0], d.contact.pos[:, 1]))
    d.contact.pos[:] = d.contact.pos[idx]
    d.contact.frame[:] = d.contact.frame[idx]
    d.contact.dist[:] = d.contact.dist[idx]

    for field in dataclasses.fields(Contact):
      _assert_attr_eq(dx.contact, d.contact, field.name, 'capsule_convex', 1e-4)

  def test_capsule_convex_face_deep(self):
    """Tests deep face penetration."""
    xml = self._CAP_BOX.replace('<body pos="0 0 0.54">', '<body pos="0 0 0.4">')

    _, dx = _collide(xml)
    self.assertTrue((dx.contact.dist < 0).all())
    np.testing.assert_array_almost_equal(
        dx.contact.pos, np.array([[0.5, 0, 0.425], [-0.4, 0, 0.425]])
    )
    np.testing.assert_array_almost_equal(
        dx.contact.dist, np.array([-0.15, -0.15])
    )
    np.testing.assert_array_almost_equal(
        dx.contact.frame[:, 0], np.array([[0, 0, -1]] * 2)
    )

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
    """Tests edge contact."""
    d, dx = _collide(self._CAP_EDGE_BOX)

    c = dx.contact
    self.assertEqual(c.pos.shape[0], 2)
    self.assertGreater(c.dist[1], 0)
    # extract the contact point with penetration
    c = jax.tree_util.tree_map(lambda x: x[:1], dx.contact)
    c = c.replace(dim=c.dim[:1], efc_address=c.efc_address[:1])
    for field in dataclasses.fields(Contact):
      _assert_attr_eq(c, d.contact, field.name, 'capsule_convex_edge', 1e-4)

  def test_capsule_convex_edge_deep(self):
    """Tests deep edge penetration."""
    xml = self._CAP_EDGE_BOX.replace(
        '<body pos="0.5 0 0.55"', '<body pos="0.5 0 0.42"'
    )
    _, dx = _collide(xml)

    np.testing.assert_array_equal(dx.contact.dist < 0, np.array([True, False]))
    np.testing.assert_array_almost_equal(dx.contact.dist[0], np.array([-0.13]))
    np.testing.assert_array_almost_equal(
        dx.contact.pos[0], np.array([0.5, 0, 0.435]), decimal=3
    )
    np.testing.assert_array_almost_equal(
        dx.contact.frame[0, 0], np.array([0, 0, -1]), decimal=3
    )

  def test_capsule_convex_edge_shallow_tip(self):
    """Tests shallow edge penetration on the tip of the capsule."""
    # the capsule sphere is inside the edge voronoi region, so there is an
    # edge contact
    xml = self._CAP_EDGE_BOX.replace(
        '<geom fromto="-0.6 0 0 0.6 0 0" size="0.05"',
        '<geom fromto="0.6 0 0.6 -0.05 0 0" size="0.1"',
    )
    xml = xml.replace('<body pos="0.5 0 0.55"', '<body pos="0.58 0 0.55"')
    d, dx = _collide(xml)

    c = dx.contact
    self.assertEqual(c.pos.shape[0], 2)
    self.assertGreater(c.dist[1], 0)
    # extract the contact point with penetration
    c = jax.tree_util.tree_map(lambda x: x[:1], dx.contact)
    c = c.replace(dim=c.dim[:1], efc_address=c.efc_address[:1])
    for field in dataclasses.fields(Contact):
      _assert_attr_eq(c, d.contact, field.name, 'edge_shallow_tip1', 1e-4)
    np.testing.assert_array_almost_equal(
        dx.contact.frame[0][0, :3], np.array([-0.43952, 0.0, -0.898233])
    )

    # the capsule sphere is outside the edge voronoi region, so there is a
    # face contact
    xml = self._CAP_EDGE_BOX.replace(
        '<geom fromto="-0.6 0 0 0.6 0 0" size="0.05"',
        '<geom fromto="-0.6 0 0.6 -0.05 0 0" size="0.1"',
    )
    xml = xml.replace('<body pos="0.5 0 0.55"', '<body pos="0.5 0 0.52"')
    d, dx = _collide(xml)

    c = dx.contact
    self.assertEqual(c.pos.shape[0], 2)
    self.assertGreater(c.dist[1], 0)
    # extract the contact point with penetration
    c = jax.tree_util.tree_map(lambda x: x[:1], dx.contact)
    c = c.replace(dim=c.dim[:1], efc_address=c.efc_address[:1])
    for field in dataclasses.fields(Contact):
      _assert_attr_eq(c, d.contact, field.name, 'edge_shallow_tip2', 1e-4)
    np.testing.assert_array_almost_equal(
        dx.contact.frame[0][0, :3], np.array([0.0, 0.0, -1.0])
    )


class CylinderTest(absltest.TestCase):
  """Tests the cylinder contact functions."""

  _CYLINDER_PLANE = """
    <mujoco>
      <worldbody>
        <geom size="40 40 40" type="plane"/>
        <body pos="0 0 0.04">
          <joint type="free"/>
          <geom fromto="-0.1 0 0 0.1 0 0" size="0.05" type="cylinder"/>
        </body>
      </worldbody>
    </mujoco>
  """

  def test_cylinder_plane(self):
    d, dx = _collide(self._CYLINDER_PLANE)

    # cylinder is lying flat
    np.testing.assert_array_less(dx.contact.dist[:2], 0)
    np.testing.assert_array_less(-dx.contact.dist[2:], 0)

    # sort position for comparison
    idx = np.lexsort((dx.contact.pos[:, 0], dx.contact.pos[:, 1]))
    dx = dx.tree_replace({'contact.pos': dx.contact.pos[idx]})
    idx = np.lexsort((d.contact.pos[:, 0], d.contact.pos[:, 1]))
    d.contact.pos[:] = d.contact.pos[idx]

    # extract the contact points with penetration
    c = jax.tree_util.tree_map(lambda x: x[:2], dx.contact)
    c = c.replace(dim=c.dim[:2], efc_address=c.efc_address[:2])
    for field in dataclasses.fields(Contact):
      _assert_attr_eq(c, d.contact, field.name, 'cylinder_plane', 1e-5)

    # cylinder is vertical
    xml = self._CYLINDER_PLANE.replace(
        '<geom fromto="-0.1 0 0 0.1 0 0"', '<geom fromto="0 0 -0.1 0 0 0.1"')
    xml = xml.replace('pos="0 0 0.04"', 'pos="0 0 0.095"')
    d, dx = _collide(xml)

    np.testing.assert_array_less(dx.contact.dist, 0)
    for field in dataclasses.fields(Contact):
      _assert_attr_eq(dx.contact, d.contact, field.name, 'cylinder_plane', 1e-5)


class ConvexTest(absltest.TestCase):
  """Tests the convex contact functions."""

  _BOX_PLANE = """
    <mujoco>
      <worldbody>
        <geom size="40 40 40" type="plane"/>
        <body pos="0 0 0.7" euler="45 0 0">
          <freejoint/>
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
    c = jax.tree_util.tree_map(lambda x: jp.take(x, jp.array([0, 1]), axis=0), dx.contact)
    c = c.replace(dim=c.dim[[0, 1]], efc_address=c.efc_address[[0, 1]])
    for field in dataclasses.fields(Contact):
      _assert_attr_eq(c, d.contact, field.name, 'box_plane', 1e-5)

  _FLAT_BOX_PLANE = """
    <mujoco>
      <worldbody>
        <geom size="40 40 40" type="plane"/>
        <body pos="0 0 0.45">
          <freejoint/>
          <geom size="0.5 0.5 0.5" type="box"/>
        </body>
      </worldbody>
    </mujoco>
  """

  def test_flat_box_plane(self):
    """Tests box collision with a plane."""
    d, dx = _collide(self._FLAT_BOX_PLANE)

    np.testing.assert_array_less(dx.contact.dist, 0)

    # sort positions for comparison
    idx = np.lexsort((dx.contact.pos[:, 0], dx.contact.pos[:, 1]))
    dx = dx.tree_replace({'contact.pos': dx.contact.pos[idx]})
    idx = np.lexsort((d.contact.pos[:, 0], d.contact.pos[:, 1]))
    d.contact.pos[:] = d.contact.pos[idx]

    for field in dataclasses.fields(Contact):
      _assert_attr_eq(dx.contact, d.contact, field.name, 'flat_box_plane', 1e-5)

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
    c = jax.tree_util.tree_map(lambda x: x[:1], dx.contact)
    c = c.replace(dim=c.dim[:1], efc_address=c.efc_address[:1])
    for field in dataclasses.fields(Contact):
      _assert_attr_eq(c, d.contact, field.name, 'box_box_edge', 1e-2)

  _CONVEX_CONVEX = """
    <mujoco>
      <asset>
        <mesh name="dodecahedron" file="meshes/dodecahedron.stl" scale="0.01 0.01 0.01" />
      </asset>
      <worldbody>
        <body pos="0.0 2.0 0.096">
          <joint axis="1 0 0" type="free"/>
          <geom size="0.2 0.2 0.2" type="mesh" mesh="dodecahedron"/>
        </body>
        <body pos="0.0 2.0 0.281" euler="0.1 -0.1 45">
          <joint axis="1 0 0" type="free"/>
          <geom size="0.1 0.1 0.1" type="mesh" mesh="dodecahedron"/>
        </body>
      </worldbody>
    </mujoco>
  """

  def test_convex_convex(self):
    """Tests generic convex-convex collision via _sat_gaussmap."""
    directory = epath.resource_path('mujoco.mjx')
    assets = {
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

  _CONVEX_CONVEX_THIN = """
    <mujoco>
      <asset>
        <mesh name="poly"
         vertex="0.3 0 0  0 0.5 0  -0.3 0 0  0 -0.5 0  0 -1 1  0 1 1"
         face="0 1 5  0 5 4  0 4 3  3 4 2  2 4 5  1 2 5  0 2 1  0 3 2"/>
      </asset>
      <worldbody>
        <body pos="0.0 2.0 0.35" euler="0 0 90">
          <freejoint/>
          <geom size="0.2 0.2 0.2" type="mesh" mesh="poly"/>
        </body>
        <body pos="0.0 2.0 2.281" euler="180 0 0">
          <freejoint/>
          <geom size="0.2 0.2 0.2" type="mesh" mesh="poly"/>
        </body>
      </worldbody>
    </mujoco>
  """

  def test_convex_convex_edge(self):
    """Tests convex-convex collisions with edge contact via _sat_gaussmap."""
    _, dx = _collide(self._CONVEX_CONVEX_THIN)
    c = dx.contact

    # Only one contact point for an edge contact.
    self.assertLess(c.dist[0], 0)
    np.testing.assert_array_less(0, c.dist[1:])
    np.testing.assert_array_almost_equal(c.frame[0, 0], np.array([0, 0, 1]))
    np.testing.assert_array_almost_equal(
        c.pos[0], np.array([0, 2, 1.3155]), decimal=5)

    _, dx = _collide(
        self._CONVEX_CONVEX_THIN.replace(
            'pos="0.0 2.0 0.35"', 'pos="0.0 2.0 0"'))
    c = dx.contact
    self.assertTrue((c.dist > 0).all())


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


class DimTest(parameterized.TestCase):
  """Tests contact dim."""

  def test_ncon(self):
    m = test_util.load_test_file('constraints.xml')
    dim = collision_driver.make_condim(m)
    expected = [1] * 4 + [3] * 20 + [4] * 4 + [6] * 4
    np.testing.assert_array_equal(dim, np.array(expected))

  def test_disable_contact(self):
    m = test_util.load_test_file('constraints.xml')
    m.opt.disableflags |= DisableBit.CONTACT
    dim = collision_driver.make_condim(m)
    self.assertEqual(dim.size, 0)

  def test_ncon_meshes(self):
    m = test_util.load_test_file('shadow_hand/scene_right.xml')

    ncon = collision_driver.make_condim(m).size
    self.assertEqual(ncon, 15)

    mx = mjx.put_model(m)
    ncon = collision_driver.make_condim(mx).size
    self.assertEqual(ncon, 15)

    # get rid of max_contact_points, test only max_geom_pairs
    for i in range(m.nnumeric):
      name_ = (
          m.names[m.name_numericadr[i] :].decode('utf-8').split('\x00', 1)[0]
      )
      if name_ == 'max_contact_points':
        m.numeric_data[m.numeric_adr[i]] = -1

    ncon = collision_driver.make_condim(m).size
    self.assertEqual(ncon, 98)

    mx = mjx.put_model(m)
    ncon = collision_driver.make_condim(mx).size
    self.assertEqual(ncon, 98)


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

  _CAPSULES_MAX_PAIR = """
    <mujoco>
      <custom>
        <numeric data="2" name="max_geom_pairs"/>
      </custom>
      <worldbody>
        <body pos="0 0 0.54">
          <freejoint/>
          <geom fromto="-0.4 0 0 0.4 0 0" size="0.05" type="capsule"/>
        </body>
        <body pos="0 0 0.54">
          <freejoint/>
          <geom fromto="-0.4 0 0 0.4 0 0" size="0.05" type="capsule"/>
        </body>
        <body pos="0 0 0.54">
          <freejoint/>
          <geom fromto="-0.4 0 0 0.4 0 0" size="0.05" type="capsule"/>
        </body>
        <body pos="0 0 1.0">
          <freejoint/>
          <geom fromto="-0.4 0 0 0.4 0 0" size="0.05" type="capsule"/>
        </body>
      </worldbody>
    </mujoco>
  """

  def test_max_pair(self):
    """Tests contact culling before the collision functions were dispatched."""
    m = mujoco.MjModel.from_xml_string(self._CAPSULES_MAX_PAIR)
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

    self.assertEqual(dx_all.contact.dist.shape, (6,))
    self.assertEqual(dx_top_k.contact.dist.shape, (2,))
    self.assertTrue((dx_top_k.contact.dist < 0).all())


if __name__ == '__main__':
  absltest.main()
