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

"""Tests for sensor functions."""

import itertools

import mujoco
import numpy as np
import warp as wp
from absl.testing import absltest
from absl.testing import parameterized

import mujoco_warp as mjwarp

from mujoco.mjx.third_party.mujoco_warp._src import test_util

# tolerance for difference between MuJoCo and MJWarp calculations - mostly
# due to float precision
_TOLERANCE = 5e-5


def _assert_eq(a, b, name):
  tol = _TOLERANCE * 10  # avoid test noise
  err_msg = f"mismatch: {name}"
  np.testing.assert_allclose(a, b, err_msg=err_msg, atol=tol, rtol=tol)


class SensorTest(parameterized.TestCase):
  def test_sensor(self):
    """Test sensors."""
    mjm, mjd, m, d = test_util.fixture(
      xml="""
      <mujoco>
        <option gravity="-1 -1 -1"/>
        <worldbody>
          <body name="body0" pos="0.1 0.2 0.3" quat=".05 .1 .15 .2">
            <joint name="slide" type="slide"/>
            <geom name="geom0" type="sphere" size="0.1"/>
            <site name="site0"/>
            <camera name="cam0"/>
          </body>
          <body name="body1" pos=".5 .6 .7">
            <joint name="ballquat" type="ball"/>
            <geom type="sphere" size="0.1"/>
            <site name="site1" pos=".1 .2 .3"/>
          </body>
          <body name="body2" pos="1 1 1">
            <freejoint/>
            <geom type="sphere" size="0.1"/>
            <site name="site2"/>
          </body>
          <body name="body3" pos="2 2 2">
            <joint name="hinge0" type="hinge" axis="1 0 0"/>
            <geom type="sphere" size="0.1" pos=".1 0 0"/>
            <body pos="2 2 2">
              <joint name="hinge1" type="hinge" axis="1 0 0"/>
              <geom type="sphere" size="0.1" pos=".1 0 0"/>
            </body>
          </body>
          <body name="body4" pos="1 0 0">
            <joint type="ball"/>
            <geom type="sphere" size="0.1" pos=".1 0 0"/>
            <body>
              <joint type="ball"/>
              <geom type="sphere" size="0.1" pos=".1 0 0"/>
            </body>
          </body>
          <body pos="10 0 0">
            <joint type="hinge" axis="1 2 3"/>
            <geom type="sphere" size="0.1"/>
            <site name="force_site" pos="1 2 3"/>
          </body>
          <body pos="20 0 0">
            <joint type="slide" axis="1 2 3"/>
            <geom type="sphere" size="0.1"/>
            <site name="torque_site" pos="1 2 3"/>
          </body>
          <body name="body8">
            <joint type="hinge"/>
            <geom type="sphere" size="0.1" pos="1 2 3"/>
            <body name="body9">
              <joint type="hinge"/>
              <geom name="geom9" type="sphere" size="0.1" pos="1 2 3"/>
              <site name="site9" pos=".2 .4 .6"/>
            </body>
          </body>
          <camera name="camera"/>
          <site name="camera_site" pos="0 0 -1"/>
          <!-- limit pos: slide -->
          <body pos="1 1 1">
            <joint name="limitslide" type="slide" limited="true" range="-.5 .5" margin=".1"/>
            <geom type="sphere" size=".1"/>
          </body>
          <!-- limit pos: hinge -->
          <body pos="2 2 2">
            <joint name="limithinge" type="hinge" limited="true" range="-.4 .4" margin=".09"/>
            <geom type="sphere" size=".1"/>
          </body>
          <!-- limit pos: ball -->
          <body pos="3 3 3">
            <joint name="limitball" type="ball" limited="true" range="0 .1" margin=".05"/>
            <geom type="sphere" size=".1"/>
          </body>
          <!-- tendon limit pos -->
          <site name="sitetendon0" pos="5 4 4"/>
          <body pos="4 4 4">
            <joint type="slide" axis="1 0 0"/>
            <geom type="sphere" size=".1"/>
            <site name="sitetendon1"/>
          </body>
          <body pos="3 3 3">
            <joint name="tendonjoint" type="hinge"/>
            <geom type="sphere" size=".1"/>
          </body>
        </worldbody>
        <tendon>
          <spatial name="limittendon" limited="true" range="0 .5" margin=".1">
            <site site="sitetendon0"/>
            <site site="sitetendon1"/>
          </spatial>
          <fixed name="tendon">
            <joint joint="tendonjoint" coef=".123"/>
          </fixed>
        </tendon>
        <actuator>
          <motor name="slide" joint="slide"/>
          <motor tendon="tendon"/>
          <motor tendon="tendon" gear="2"/>
        </actuator>
        <sensor>
          <magnetometer site="site0"/>
          <camprojection camera="camera" site="camera_site"/>
          <camprojection camera="camera" site="camera_site" cutoff=".001"/>
          <jointpos joint="slide"/>
          <jointpos joint="slide" cutoff=".001"/>
          <actuatorpos actuator="slide"/>
          <actuatorpos actuator="slide" cutoff=".001"/>
          <ballquat joint="ballquat"/>
          <jointlimitpos joint="limitslide"/>
          <jointlimitpos joint="limitslide" cutoff=".001"/>
          <jointlimitpos joint="limithinge"/>
          <jointlimitpos joint="limithinge" cutoff=".001"/>
          <jointlimitpos joint="limitball"/>
          <jointlimitpos joint="limitball" cutoff=".001"/>
          <tendonlimitpos tendon="limittendon"/>
          <tendonlimitpos tendon="limittendon" cutoff=".001"/>
          <framepos objtype="body" objname="body1"/>
          <framepos objtype="body" objname="body1" cutoff=".001"/>
          <framepos objtype="body" objname="body1" reftype="body" refname="body0"/>
          <framepos objtype="body" objname="body1" reftype="geom" refname="geom0"/>
          <framepos objtype="body" objname="body1" reftype="site" refname="site0"/>
          <framepos objtype="body" objname="body1" reftype="cam" refname="cam0"/>
          <framepos objtype="xbody" objname="body1"/>
          <framepos objtype="geom" objname="geom0"/>
          <framepos objtype="site" objname="site0"/>
          <framepos objtype="camera" objname="cam0"/>
          <framexaxis objtype="body" objname="body1"/>
          <framexaxis objtype="body" objname="body1" reftype="body" refname="body0"/>
          <framexaxis objtype="body" objname="body1" reftype="geom" refname="geom0"/>
          <framexaxis objtype="body" objname="body1" reftype="site" refname="site0"/>
          <framexaxis objtype="body" objname="body1" reftype="cam" refname="cam0"/>
          <framexaxis objtype="xbody" objname="body1"/>
          <framexaxis objtype="geom" objname="geom0"/>
          <framexaxis objtype="site" objname="site0"/>
          <framexaxis objtype="camera" objname="cam0"/>
          <frameyaxis objtype="body" objname="body1"/>
          <frameyaxis objtype="body" objname="body1" reftype="body" refname="body0"/>
          <frameyaxis objtype="body" objname="body1" reftype="geom" refname="geom0"/>
          <frameyaxis objtype="body" objname="body1" reftype="site" refname="site0"/>
          <frameyaxis objtype="body" objname="body1" reftype="cam" refname="cam0"/>
          <frameyaxis objtype="xbody" objname="body1"/>
          <frameyaxis objtype="geom" objname="geom0"/>
          <frameyaxis objtype="site" objname="site0"/>
          <frameyaxis objtype="camera" objname="cam0"/>
          <framezaxis objtype="body" objname="body1"/>
          <framezaxis objtype="body" objname="body1" reftype="body" refname="body0"/>
          <framezaxis objtype="body" objname="body1" reftype="geom" refname="geom0"/>
          <framezaxis objtype="body" objname="body1" reftype="site" refname="site0"/>
          <framezaxis objtype="body" objname="body1" reftype="cam" refname="cam0"/>
          <framezaxis objtype="xbody" objname="body1"/>
          <framezaxis objtype="geom" objname="geom0"/>
          <framezaxis objtype="site" objname="site0"/>
          <framezaxis objtype="camera" objname="cam0"/>
          <framequat objtype="body" objname="body1"/>
          <framequat objtype="body" objname="body1" reftype="body" refname="body0"/>
          <framequat objtype="body" objname="body1" reftype="geom" refname="geom0"/>
          <framequat objtype="body" objname="body1" reftype="site" refname="site0"/>
          <framequat objtype="body" objname="body1" reftype="cam" refname="cam0"/>
          <framequat objtype="xbody" objname="body1"/>
          <framequat objtype="geom" objname="geom0"/>
          <framequat objtype="site" objname="site0"/>
          <framequat objtype="camera" objname="cam0"/>
          <subtreecom body="body3"/>
          <subtreecom body="body3" cutoff=".001"/>
          <e_potential/>
          <e_potential cutoff=".001"/>
          <e_kinetic/>
          <e_kinetic cutoff=".001"/>
          <clock/>
          <clock cutoff=".001"/>
          <velocimeter site="site2"/>
          <velocimeter site="site2" cutoff=".001"/>
          <gyro site="site2"/>
          <gyro site="site2" cutoff=".001"/>
          <jointvel joint="slide"/>
          <jointvel joint="slide" cutoff=".001"/>
          <actuatorvel actuator="slide"/>
          <actuatorvel actuator="slide" cutoff=".001"/>
          <ballangvel joint="ballquat"/>
          <ballangvel joint="ballquat" cutoff=".001"/>
          <jointlimitvel joint="limithinge"/>
          <jointlimitvel joint="limithinge" cutoff=".001"/>
          <tendonlimitvel tendon="limittendon"/>
          <tendonlimitvel tendon="limittendon" cutoff=".001"/>
          <framelinvel objtype="body" objname="body9"/>
          <framelinvel objtype="body" objname="body9" cutoff=".001"/>
          <frameangvel objtype="body" objname="body9"/>
          <frameangvel objtype="body" objname="body9" cutoff=".001"/>
          <framelinvel objtype="xbody" objname="body9"/>
          <frameangvel objtype="xbody" objname="body9"/>
          <framelinvel objtype="geom" objname="geom9"/>
          <frameangvel objtype="geom" objname="geom9"/>
          <framelinvel objtype="site" objname="site9"/>
          <frameangvel objtype="site" objname="site9"/>
          <framelinvel objtype="camera" objname="cam0"/>
          <frameangvel objtype="camera" objname="cam0"/>
          <framelinvel objtype="body" objname="body9" reftype="xbody" refname="body0"/>
          <frameangvel objtype="body" objname="body9" reftype="xbody" refname="body0"/>
          <framelinvel objtype="body" objname="body9" reftype="geom" refname="geom0"/>
          <frameangvel objtype="body" objname="body9" reftype="geom" refname="geom0"/>
          <framelinvel objtype="body" objname="body9" reftype="site" refname="site0"/>
          <frameangvel objtype="body" objname="body9" reftype="site" refname="site0"/>
          <framelinvel objtype="body" objname="body9" reftype="cam" refname="cam0"/>
          <frameangvel objtype="body" objname="body9" reftype="cam" refname="cam0"/>
          <subtreelinvel body="body4"/>
          <subtreelinvel body="body4" cutoff=".001"/>
          <subtreeangmom body="body4"/>
          <subtreeangmom body="body4" cutoff=".001"/>
          <accelerometer site="force_site"/>
          <accelerometer site="force_site" cutoff=".001"/>
          <force site="force_site"/>
          <force site="force_site" cutoff=".001"/>
          <torque site="torque_site"/>
          <torque site="torque_site" cutoff=".001"/>
          <actuatorfrc actuator="slide"/>
          <actuatorfrc actuator="slide" cutoff=".001"/>
          <jointactuatorfrc joint="slide"/>
          <jointactuatorfrc joint="slide" cutoff=".001"/>
          <jointlimitfrc joint="limitslide"/>
          <jointlimitfrc joint="limitslide" cutoff=".001"/>
          <jointlimitfrc joint="limithinge"/>
          <jointlimitfrc joint="limithinge" cutoff=".001"/>
          <jointlimitfrc joint="limitball"/>
          <jointlimitfrc joint="limitball" cutoff=".001"/>
          <tendonlimitfrc tendon="limittendon"/>
          <tendonlimitfrc tendon="limittendon" cutoff=".001"/>
          <tendonactuatorfrc tendon="tendon"/>
          <tendonactuatorfrc tendon="tendon" cutoff=".001"/>
          <framelinacc objtype="body" objname="body9"/>
          <framelinacc objtype="body" objname="body9" cutoff=".001"/>
          <frameangacc objtype="body" objname="body9"/>
          <frameangacc objtype="body" objname="body9" cutoff=".001"/>
          <framelinacc objtype="xbody" objname="body9"/>
          <frameangacc objtype="xbody" objname="body9"/>
          <framelinacc objtype="geom" objname="geom9"/>
          <frameangacc objtype="geom" objname="geom9"/>
          <framelinacc objtype="site" objname="site9"/>
          <frameangacc objtype="site" objname="site9"/>
          <framelinacc objtype="camera" objname="cam0"/>
          <frameangacc objtype="camera" objname="cam0"/>
        </sensor>
        <keyframe>
          <key qpos="1 .1 .2 .3 .4 1 1 1 1 0 0 0 .25 .35 1 0 0 0 1 0 0 0 0 0 1 1 .6 .5 1 2 3 4 .5 0" qvel="2 .2 -.1 .4 .25 .35 .45 -0.1 -0.2 -0.3 .1 -.2 -.5 -0.75 -1 .1 .2 .3 0 0 2 2 0 0 0 0 0 0 0" ctrl="3 .1 .2"/>
        </keyframe>
      </mujoco>
    """,
      keyframe=0,
      kick=True,
    )

    d.sensordata.zero_()

    mjwarp.sensor_pos(m, d)
    mjwarp.sensor_vel(m, d)
    mjwarp.sensor_acc(m, d)

    _assert_eq(d.sensordata.numpy()[0], mjd.sensordata, "sensordata")

  def test_rangefinder(self):
    """Test rangefinder."""
    for keyframe in range(2):
      _, mjd, m, d = test_util.fixture(
        xml="""
        <mujoco>
          <compiler angle="degree"/>
          <worldbody>
            <geom type="sphere" size=".1" pos="0 0 1"/>
            <body>
              <joint name="joint0" type="hinge" axis="1 0 0"/>
              <joint type="hinge" axis="0 1 0"/>
              <joint type="hinge" axis="0 0 1"/>
              <geom type="sphere" size="0.1"/>
              <site name="site0" size=".1"/>
              <site name="site1" size=".1" pos="0 0 -.2"/>
            </body>
          </worldbody>
          <sensor>
            <rangefinder site="site0"/>
            <jointpos joint="joint0"/>
            <rangefinder site="site1"/>
          </sensor>
          <keyframe>
            <key qpos="0 0 0"/>
            <key qpos="0 90 0"/>
          </keyframe>
        </mujoco>
      """,
        keyframe=keyframe,
      )

      d.sensordata.zero_()

      mjwarp.sensor_pos(m, d)

      _assert_eq(d.sensordata.numpy()[0], mjd.sensordata, "sensordata")

  def test_touch_sensor(self):
    """Test touch sensor."""
    for keyframe in range(2):
      _, mjd, m, d = test_util.fixture(
        xml="""
        <mujoco>
          <worldbody>
            <geom type="plane" size="10 10 .001"/>
            <body pos="0 0 .25">
              <geom type="sphere" size="0.1" pos=".1 0 0"/>
              <geom type="sphere" size="0.1" pos="-.1 0 0"/>
              <geom type="sphere" size="0.1" pos="0 0 .11"/>
              <geom type="sphere" size="0.1" pos="0 0 10"/>
              <site name="site_sphere" type="sphere" size=".2"/>
              <site name="site_capsule" type="capsule" size=".2 .2"/>
              <site name="site_ellipsoid" type="ellipsoid" size=".2 .2 .2"/>
              <site name="site_cylinder" type="cylinder" size=".2 .2"/>
              <site name="site_box" type="box" size=".2 .2 .2"/>
              <freejoint/>
            </body>
          </worldbody>
          <sensor>
            <touch site="site_sphere"/>
            <touch site="site_capsule"/>
            <touch site="site_ellipsoid"/>
            <touch site="site_cylinder"/>
            <touch site="site_box"/>
          </sensor>
          <keyframe>
            <key qpos="0 0 10 1 0 0 0"/>
            <key qpos="0 0 .05 1 0 0 0"/>
            <key qpos="0 0 0 1 0 0 0"/>
          </keyframe>
        </mujoco>
      """,
        keyframe=keyframe,
      )

      d.sensordata.zero_()

      mjwarp.sensor_acc(m, d)

      _assert_eq(d.sensordata.numpy()[0], mjd.sensordata, "sensordata")

  def test_tendon_sensor(self):
    """Test tendon sensors."""
    _, mjd, m, d = test_util.fixture("tendon/fixed.xml", keyframe=0, sparse=False)

    d.sensordata.zero_()

    mjwarp.sensor_pos(m, d)
    mjwarp.sensor_vel(m, d)

    _assert_eq(d.sensordata.numpy()[0], mjd.sensordata, "sensordata")

  @parameterized.parameters("humanoid/humanoid.xml", "constraints.xml")
  def test_energy(self, xml):
    mjm, mjd, m, d = test_util.fixture(xml, constraint=False, kick=True)

    d.energy.zero_()

    mujoco.mj_energyPos(mjm, mjd)
    mjwarp.energy_pos(m, d)

    _assert_eq(d.energy.numpy()[0][0], mjd.energy[0], "potential energy")

    mujoco.mj_energyVel(mjm, mjd)
    mjwarp.energy_vel(m, d)

    _assert_eq(d.energy.numpy()[0][1], mjd.energy[1], "kinetic energy")

  @parameterized.parameters(
    'type="capsule" size=".1 .1" euler="0 89 89"',
    'type="box" size=".1 .11 .12" euler=".02 .05 .1"',
  )
  def test_contact_sensor(self, geom):
    """Test contact sensor."""
    # create contact sensors
    contact_sensor = ""

    # data combinations
    datas = ["found", "force dist normal", "torque pos tangent", "found force torque dist pos normal tangent"]

    for geoms in [
      'geom2="plane"',
      'geom1="plane" geom2="geom"',
      'geom1="geom" geom2="plane"',
      'body1="plane"',
      'body1="plane" body2="geom"',
      'body1="geom" body2="plane"',
    ]:
      for num in [1, 5]:
        for reduce in [None, "mindist", "maxforce"]:
          for data in datas:
            contact_sensor += f'<contact {geoms} num="{num}"'
            if reduce is not None:
              contact_sensor += f' reduce="{reduce}"'
            contact_sensor += f' data="{data}"/>'

    _MJCF = f"""
    <mujoco>
      <compiler angle="degree"/>
      <option cone="pyramidal"/>
      <worldbody>
        <body name="plane">
          <geom name="plane" type="plane" size="10 10 .001"/>
        </body>
        <body name="geom">
          <geom name="geom" {geom}/>
          <joint type="slide" axis="0 0 1"/>
        </body>
        <body name="sphere">
          <geom name="sphere" type="sphere" size=".1"/>
          <joint type="slide" axis="0 0 1"/>
        </body>
      </worldbody>
      <keyframe>
        <key qpos=".08 1"/>
      </keyframe>
      <sensor>
        {contact_sensor}
      </sensor>
    </mujoco>
    """

    _, mjd, m, d = test_util.fixture(xml=_MJCF, keyframe=0)

    d.sensordata.zero_()
    mjwarp.forward(m, d)

    print(f"ncon: {d.ncon.numpy()}")

    sensordata = d.sensordata.numpy()[0]
    _assert_eq(sensordata, mjd.sensordata, "sensordata")
    self.assertTrue(sensordata.any())  # check that sensordata is not empty

  def test_contact_sensor_subtree(self):
    """Test contact sensor with subtree matching semantics."""
    _MJCF = f"""
    <mujoco>
      <worldbody>
        <geom type="plane" size="2 2 .01"/>
        <body name="thigh" pos="-1 0 .1">
          <joint type="slide"/>
          <geom type="capsule" size=".1" fromto="0 0 0 .5 0 0"/>
          <body name="shin" pos=".7 0 0">
            <joint axis="0 1 0"/>
            <geom type="capsule" size=".1" fromto="0 0 0 0 0 .5"/>
            <body name="foot" pos="0 0 .7">
              <joint axis="0 1 0"/>
              <geom type="capsule" size=".1" fromto="0 0 0 -.7 0 0"/>
            </body>
          </body>
        </body>
      </worldbody>
      <sensor>
        <contact name="all" reduce="mindist"/>
        <contact name="world" subtree1="world" reduce="mindist"/>
        <contact name="thigh" subtree1="thigh" reduce="mindist"/>
        <contact name="shin" subtree1="shin" reduce="mindist"/>
        <contact name="foot" subtree1="foot" reduce="mindist"/>
        <contact name="foot_w" subtree1="foot" body2="world" reduce="mindist"/>
        <contact name="foot_w2" subtree1="foot" subtree2="world" reduce="mindist"/>
      </sensor>
      <keyframe>
        <key qpos="-6.96651e-05 -0.0478055 -0.746498"/>
      </keyframe>
    </mujoco>
    """
    _, _, m, d = test_util.fixture(xml=_MJCF, nconmax=12, njmax=48, keyframe=0)

    d.sensordata.zero_()
    mjwarp.forward(m, d)

    _assert_eq(d.sensordata.numpy()[0], np.array([4, 4, 4, 2, 1, 0, 1]), "found")

  def test_contact_sensor_subtree_found_zero(self):
    """Test contact sensor with subtree matching semantics generates found=0."""
    _MJCF = f"""
    <mujoco>
      <worldbody>
        <geom type="plane" size="2 2 .01"/>
        <body name="base">
          <joint type="slide" axis="1 0 0"/>
          <joint type="slide" axis="0 0 1"/>
          <geom type="capsule" size=".1" fromto="0 0 0 1 0 0"/>
          <body name="body0">
            <joint axis="0 1 0"/>
            <geom type="capsule" size=".1" fromto="0 0 0 0 0 -1"/>
          </body>
          <body name="body1" pos="1 0 0">
            <joint axis="0 1 0"/>
            <geom type="capsule" size=".1" fromto="0 0 0 0 0 -1"/>
          </body>
        </body>
      </worldbody>
      <sensor>
        <contact subtree1="base" subtree2="base"/>
      </sensor>
      <keyframe>
        <key qpos="0 1 0 0"/>
      </keyframe>
    </mujoco>
    """
    _, _, m, d = test_util.fixture(xml=_MJCF, nconmax=2, njmax=8, keyframe=0)

    d.sensordata.fill_(wp.inf)
    mjwarp.forward(m, d)

    _assert_eq(d.ncon.numpy()[0], 2, "ncon")
    _assert_eq(d.sensordata.numpy()[0], 0, "found")

  @parameterized.product(site_geom=["sphere", "capsule", "ellipsoid", "cylinder", "box"], key_pos=["0 0 10", "0 0 .09"])
  def test_contact_sensor_site(self, site_geom, key_pos):
    _, mjd, m, d = test_util.fixture(
      xml=f"""
    <mujoco>
      <worldbody>
        <geom type="plane" size="10 10 .001"/>
        <site name="site" type="{site_geom}" size=".1 .2 .3"/>
        <body>
          <geom type="sphere" size=".1"/>
          <freejoint/>
        </body>
      </worldbody>
      <sensor>
        <contact site="site" reduce="mindist"/>
      </sensor>
      <keyframe>
        <key qpos="{key_pos} 1 0 0 0"/>
      </keyframe>
    </mujoco>
    """,
      keyframe=0,
    )

    d.sensordata.zero_()
    mjwarp.forward(m, d)

    _assert_eq(d.sensordata.numpy()[0], mjd.sensordata, "sensordata")

  def test_contact_sensor_netforce(self):
    """Test contact sensor with netforce reduction."""
    _, mjd, m, d = test_util.fixture(
      xml="""
    <mujoco>
      <worldbody>
        <geom name="plane" type="plane" size="10 10 .001"/>
        <body>
          <geom name="box" type="box" size=".1 .1 .1"/>
          <freejoint/>
        </body>
      </worldbody>
      <sensor>
        <contact geom1="plane" geom2="box" data="found force torque dist pos normal tangent" reduce="netforce" num="2"/>
        <contact geom1="box" geom2="plane" data="force torque" reduce="netforce"/>
      </sensor>
      <keyframe>
        <key qpos="0 0 .09 1 0 0 0"/>
      </keyframe>
    </mujoco>
    """,
      keyframe=0,
    )

    d.sensordata.zero_()
    mjwarp.forward(m, d)
    sensordata = d.sensordata.numpy()[0]
    _assert_eq(sensordata, mjd.sensordata, "sensordata")
    self.assertTrue(sensordata.any())  # check that sensordata is not empty


if __name__ == "__main__":
  wp.init()
  absltest.main()
