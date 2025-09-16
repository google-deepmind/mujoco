# Copyright 2025 The Newton Developers
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

"""Tests for broadphase functions."""

import numpy as np
import warp as wp
from absl.testing import absltest
from absl.testing import parameterized

import mujoco_warp as mjwarp

from mujoco.mjx.third_party.mujoco_warp._src import collision_driver
from mujoco.mjx.third_party.mujoco_warp._src import test_util
from mujoco.mjx.third_party.mujoco_warp._src.types import BroadphaseFilter
from mujoco.mjx.third_party.mujoco_warp._src.types import BroadphaseType


def broadphase_caller(m, d):
  if m.opt.broadphase == int(BroadphaseType.NXN):
    collision_driver.nxn_broadphase(m, d)
  else:
    collision_driver.sap_broadphase(m, d)


class BroadphaseTest(parameterized.TestCase):
  # filter combinations
  plane_sphere = BroadphaseFilter.PLANE.value | BroadphaseFilter.SPHERE.value
  plane_aabb = BroadphaseFilter.PLANE.value | BroadphaseFilter.AABB.value
  plane_obb = BroadphaseFilter.PLANE | BroadphaseFilter.OBB.value
  plane_sphere_aabb = plane_sphere | BroadphaseFilter.AABB.value
  plane_sphere_obb = plane_sphere | BroadphaseFilter.OBB.value
  plane_sphere_aabb_obb = plane_sphere_aabb | BroadphaseFilter.OBB.value

  @parameterized.product(
    broadphase=list(BroadphaseType),
    filter=[plane_sphere, plane_aabb, plane_obb, plane_sphere_aabb, plane_sphere_obb, plane_sphere_aabb_obb],
  )
  def test_broadphase(self, broadphase, filter):
    """Tests collision broadphase algorithms."""

    _XML = """
      <mujoco>
        <worldbody>
          <body>
            <freejoint/>
            <geom type="sphere" size="0.1"/>
          </body>
          <body>
            <freejoint/>
            <geom type="sphere" size="0.1"/>
          </body>
          <body>
            <freejoint/>
            <geom type="capsule" size="0.1 0.1"/>
          </body>
          <body>
            <freejoint/>
            <geom type="sphere" size="0.1"/>
          </body>
          <body>
            <freejoint/>
            <!-- self collision -->
            <geom type="sphere" size="0.1"/>
            <geom type="sphere" size="0.1"/>
            <!-- parent-child self collision -->
            <body>
              <geom type="sphere" size="0.1"/>
              <joint type="hinge"/>
            </body>
          </body>
        </worldbody>
        <keyframe>
          <key qpos='0 0 0 1 0 0 0
                    1 0 0 1 0 0 0
                    2 0 0 1 0 0 0
                    3 0 0 1 0 0 0
                    4 0 0 1 0 0 0
                    0'/>
          <key qpos='0 0 0 1 0 0 0
                    .05 0 0 1 0 0 0
                    2 0 0 1 0 0 0
                    3 0 0 1 0 0 0
                    4 0 0 1 0 0 0
                    0'/>
          <key qpos='0 0 0 1 0 0 0
                    .01 0 0 1 0 0 0
                    .02 0 0 1 0 0 0
                    3 0 0 1 0 0 0
                    4 0 0 1 0 0 0
                    0'/>
          <key qpos='0 0 0 1 0 0 0
                    1 0 0 1 0 0 0
                    2 0 0 1 0 0 0
                    2 0 0 1 0 0 0
                    4 0 0 1 0 0 0
                    0'/>
        </keyframe>
      </mujoco>
    """

    # one world and zero collisions
    mjm, _, m, d0 = test_util.fixture(xml=_XML, keyframe=0)

    m.opt.broadphase = broadphase
    m.opt.broadphase_filter = filter

    broadphase_caller(m, d0)
    np.testing.assert_allclose(d0.ncollision.numpy()[0], 0)

    # one world and one collision
    _, mjd1, _, d1 = test_util.fixture(xml=_XML, keyframe=1)
    broadphase_caller(m, d1)

    np.testing.assert_allclose(d1.ncollision.numpy()[0], 1)
    np.testing.assert_allclose(d1.collision_pair.numpy()[0][0], 0)
    np.testing.assert_allclose(d1.collision_pair.numpy()[0][1], 1)

    # one world and three collisions
    _, mjd2, _, d2 = test_util.fixture(xml=_XML, keyframe=2)
    broadphase_caller(m, d2)

    ncollision = d2.ncollision.numpy()[0]
    np.testing.assert_allclose(ncollision, 3)

    collision_pairs = [[0, 1], [0, 2], [1, 2]]
    for i in range(ncollision):
      self.assertTrue([d2.collision_pair.numpy()[i][0], d2.collision_pair.numpy()[i][1]] in collision_pairs)

    # two worlds and four collisions
    d3 = mjwarp.make_data(mjm, nworld=2, nconmax=512, njmax=512)
    d3.geom_xpos = wp.array(
      np.vstack([np.expand_dims(mjd1.geom_xpos, axis=0), np.expand_dims(mjd2.geom_xpos, axis=0)]),
      dtype=wp.vec3,
    )
    d3.geom_xmat = wp.array(
      np.vstack([np.expand_dims(mjd1.geom_xmat, axis=0), np.expand_dims(mjd2.geom_xmat, axis=0)]),
      dtype=wp.mat33,
    )
    broadphase_caller(m, d3)

    ncollision = d3.ncollision.numpy()[0]
    np.testing.assert_allclose(ncollision, 4)

    collision_pairs = [[[0, 1]], [[0, 1], [0, 2], [1, 2]]]
    worldids = [0, 1, 1, 1]
    for i in range(ncollision):
      worldid = d3.collision_worldid.numpy()[i]
      self.assertTrue(worldid == worldids[i])
      self.assertTrue([d3.collision_pair.numpy()[i][0], d3.collision_pair.numpy()[i][1]] in collision_pairs[worldid])

    # one world and zero collisions: contype and conaffinity incompatibility
    mjm4, _, m4, d4 = test_util.fixture(xml=_XML, keyframe=1)
    mjm4.geom_contype[:3] = 0
    m4 = mjwarp.put_model(mjm4)

    broadphase_caller(m4, d4)
    np.testing.assert_allclose(d4.ncollision.numpy()[0], 0)

    # one world and one collision: geomtype ordering
    _, _, _, d5 = test_util.fixture(xml=_XML, keyframe=3)
    broadphase_caller(m, d5)
    np.testing.assert_allclose(d5.ncollision.numpy()[0], 1)
    np.testing.assert_allclose(d5.collision_pair.numpy()[0][0], 3)
    np.testing.assert_allclose(d5.collision_pair.numpy()[0][1], 2)

  @parameterized.parameters((0, 0, 0), (0, 0.011, 1), (0.011, 0, 1), (0.00999, 0, 0), (0, 0.00999, 0), (0.00999, 0.00999, 0))
  def test_broadphase_margin(self, margin1, margin2, ncollision):
    _MJCF = f"""
      <mujoco>
        <worldbody>
          <body>
            <geom type="sphere" size=".1" margin="{margin1}"/>
            <joint type="slide" axis="1 0 0"/>
          </body>
          <body>
            <geom type="sphere" size=".1" margin="{margin2}"/>
            <joint type="slide" axis="1 0 0"/>
          </body>
        </worldbody>
        <keyframe>
          <key qpos="0 .21"/>
        </keyframe>
      </mujoco>
    """
    _, _, m, d = test_util.fixture(xml=_MJCF, keyframe=0)
    broadphase_caller(m, d)
    self.assertEqual(d.ncollision.numpy()[0], ncollision)

  @parameterized.parameters(True, False)
  def test_broadphase_filterparent(self, filterparent):
    _MJCF = """
      <mujoco>
        <worldbody>
          <body>
            <geom type="sphere" size=".1"/>
            <joint type="slide"/>
            <body>
              <geom type="sphere" size=".1"/>
              <joint type="slide"/>
            </body>
          </body>
        </worldbody>
        <keyframe>
          <key qpos="0 0"/>
        </keyframe>
      </mujoco>
    """
    _, _, m, d = test_util.fixture(xml=_MJCF, filterparent=filterparent, keyframe=0)

    broadphase_caller(m, d)
    self.assertEqual(d.ncollision.numpy()[0], 0 if filterparent else 1)

  def test_broadphase_filter(self):
    plane = BroadphaseFilter.PLANE.value
    sphere = BroadphaseFilter.SPHERE.value
    aabb = BroadphaseFilter.AABB.value
    obb = BroadphaseFilter.OBB.value
    plane_sphere = plane | sphere
    plane_aabb = plane | aabb
    plane_obb = plane | obb

    _PLANE_CAPSULE_CAPSULE = """
      <mujoco>
        <option gravity="0 0 0"/>
        <worldbody>
          <light type="directional" pos="0 0 1"/>
          <geom name="floor" size="10 10 .001" type="plane"/>
          <body>
            <geom type="capsule" size=".05 .1" rgba="0 1 0 1"/>
            <joint type="slide" axis="1 0 0"/>
            <joint type="slide" axis="0 0 1"/>
            <joint type="hinge" axis="0 1 0"/>
          </body>
          <body>
            <geom type="capsule" size=".05 .1" rgba="1 0 0 1"/>
            <joint type="slide" axis="1 0 0"/>
            <joint type="slide" axis="0 0 1"/>
            <joint type="hinge" axis="0 1 0"/>
          </body>
        </worldbody>
        <keyframe>
          <key qpos="-.5 .25 0 .5 .25 0"/>
          <key qpos="-.5 .075 1.57 .5 .25 0"/>
          <key qpos="-.075 .25 0 .075 .25 0"/>
          <key qpos="0 .25 .7853 0 .45 .7853"/>
        </keyframe>
      </mujoco>
    """

    _, _, m, d = test_util.fixture(xml=_PLANE_CAPSULE_CAPSULE, keyframe=0)
    m.opt.broadphase_filter = plane_sphere
    broadphase_caller(m, d)
    self.assertEqual(d.ncollision.numpy()[0], 0)

    _, _, m, d = test_util.fixture(xml=_PLANE_CAPSULE_CAPSULE, keyframe=0)
    m.opt.broadphase_filter = plane_aabb
    broadphase_caller(m, d)
    self.assertEqual(d.ncollision.numpy()[0], 0)

    _, _, m, d = test_util.fixture(xml=_PLANE_CAPSULE_CAPSULE, keyframe=0)
    m.opt.broadphase_filter = plane_obb
    broadphase_caller(m, d)
    self.assertEqual(d.ncollision.numpy()[0], 0)

    _, _, m, d = test_util.fixture(xml=_PLANE_CAPSULE_CAPSULE, keyframe=1)
    m.opt.broadphase_filter = plane_sphere
    broadphase_caller(m, d)
    self.assertEqual(d.ncollision.numpy()[0], 1)

    # note: collision_driver._plane_filter checks bounding sphere
    _, _, m, d = test_util.fixture(xml=_PLANE_CAPSULE_CAPSULE, keyframe=1)
    m.opt.broadphase_filter = plane
    broadphase_caller(m, d)
    self.assertEqual(d.ncollision.numpy()[0], 2)

    _, _, m, d = test_util.fixture(xml=_PLANE_CAPSULE_CAPSULE, keyframe=1)
    m.opt.broadphase_filter = plane_sphere
    broadphase_caller(m, d)
    self.assertEqual(d.ncollision.numpy()[0], 1)

    _, _, m, d = test_util.fixture(xml=_PLANE_CAPSULE_CAPSULE, keyframe=1)
    m.opt.broadphase_filter = plane_obb
    broadphase_caller(m, d)
    self.assertEqual(d.ncollision.numpy()[0], 1)

    _, _, m, d = test_util.fixture(xml=_PLANE_CAPSULE_CAPSULE, keyframe=2)
    m.opt.broadphase_filter = plane_sphere
    broadphase_caller(m, d)
    self.assertEqual(d.ncollision.numpy()[0], 1)

    _, _, m, d = test_util.fixture(xml=_PLANE_CAPSULE_CAPSULE, keyframe=2)
    m.opt.broadphase_filter = plane_aabb
    broadphase_caller(m, d)
    self.assertEqual(d.ncollision.numpy()[0], 0)

    _, _, m, d = test_util.fixture(xml=_PLANE_CAPSULE_CAPSULE, keyframe=2)
    m.opt.broadphase_filter = plane_obb
    broadphase_caller(m, d)
    self.assertEqual(d.ncollision.numpy()[0], 0)

    _, _, m, d = test_util.fixture(xml=_PLANE_CAPSULE_CAPSULE, keyframe=3)
    m.opt.broadphase_filter = plane_sphere
    broadphase_caller(m, d)
    self.assertEqual(d.ncollision.numpy()[0], 1)

    _, _, m, d = test_util.fixture(xml=_PLANE_CAPSULE_CAPSULE, keyframe=3)
    m.opt.broadphase_filter = plane_aabb
    broadphase_caller(m, d)
    self.assertEqual(d.ncollision.numpy()[0], 1)

    _, _, m, d = test_util.fixture(xml=_PLANE_CAPSULE_CAPSULE, keyframe=3)
    m.opt.broadphase_filter = plane_obb
    broadphase_caller(m, d)
    self.assertEqual(d.ncollision.numpy()[0], 0)


if __name__ == "__main__":
  wp.init()
  absltest.main()
