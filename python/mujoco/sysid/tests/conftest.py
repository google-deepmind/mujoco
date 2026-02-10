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
"""Shared fixtures for mujoco.sysid tests."""

import mujoco
from mujoco.sysid._src import parameter
from mujoco.sysid._src import timeseries
from mujoco.sysid._src.model_modifier import _infer_inertial
import numpy as np
import pytest

# ---------------------------------------------------------------------------
# Inline model XML strings — no external file dependencies
# ---------------------------------------------------------------------------

BOX_XML = """\
<mujoco model="box">
  <compiler angle="radian"/>
  <option integrator="implicitfast" timestep="0.001" cone="elliptic">
    <flag contact="enable"/>
  </option>
  <asset>
    <texture type="2d" name="groundplane" builtin="checker" width="300" height="300"/>
    <material name="groundplane" texture="groundplane"/>
  </asset>
  <worldbody>
    <body name="floor">
      <geom name="floor" type="box" size="100 100 0.01" pos="0 0 -0.01"/>
    </body>
    <body name="box" pos="0 0 0.102">
      <freejoint name="box_free"/>
      <inertial pos="0 0 0" mass="5" diaginertia="0.0333 0.0333 0.0333"/>
      <geom name="box" type="box" size="0.1 0.1 0.1"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="push_x" joint="box_free" gear="1 0 0 0 0 0"
           ctrllimited="true" ctrlrange="-20 20"/>
  </actuator>
  <contact>
    <pair name="box_floor" geom1="box" geom2="floor"
          friction="1.6 0.005 0.0001" solref="0.01 1.0"/>
  </contact>
</mujoco>
"""

ARM_XML = """\
<mujoco model="test_arm">
  <compiler angle="radian" autolimits="true"/>
  <option integrator="implicitfast">
    <flag contact="disable"/>
  </option>
  <asset>
    <texture name="tex1" type="2d" builtin="checker" width="64" height="64"/>
    <material name="mat1" texture="tex1"/>
  </asset>
  <worldbody>
    <body name="link1" pos="0 0 0.1">
      <inertial pos="0 0 0.05" mass="1.0" diaginertia="0.01 0.01 0.005"/>
      <joint name="joint1" type="hinge" axis="0 0 1" range="-3.14 3.14"/>
      <geom type="capsule" fromto="0 0 0 0 0 0.1" size="0.04"/>
      <body name="link2" pos="0 0 0.1">
        <inertial pos="0 0 0.05" mass="0.8" diaginertia="0.008 0.008 0.004"/>
        <joint name="joint2" type="hinge" axis="0 1 0" range="-3.14 3.14"/>
        <geom type="capsule" fromto="0 0 0 0 0 0.1" size="0.035"/>
        <body name="link3" pos="0 0 0.1">
          <inertial pos="0 0 0.05" mass="0.6" diaginertia="0.006 0.006 0.003"/>
          <joint name="joint3" type="hinge" axis="0 1 0" range="-3.14 3.14"/>
          <geom type="capsule" fromto="0 0 0 0 0 0.1" size="0.03"/>
          <body name="link4" pos="0 0 0.1">
            <inertial pos="0 0 0.04" mass="0.4" diaginertia="0.004 0.004 0.002"/>
            <joint name="joint4" type="hinge" axis="0 0 1" range="-3.14 3.14"/>
            <geom type="capsule" fromto="0 0 0 0 0 0.08" size="0.025"/>
            <body name="link5" pos="0 0 0.08">
              <inertial pos="0 0 0.03" mass="0.2" diaginertia="0.002 0.002 0.001"/>
              <joint name="joint5" type="hinge" axis="0 1 0" range="-3.14 3.14"/>
              <geom type="capsule" fromto="0 0 0 0 0 0.06" size="0.02"/>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position name="actuator1" joint="joint1" kp="400" kv="40"/>
    <position name="actuator2" joint="joint2" kp="400" kv="40"/>
    <position name="actuator3" joint="joint3" kp="400" kv="40"/>
    <position name="actuator4" joint="joint4" kp="400" kv="40"/>
    <position name="actuator5" joint="joint5" kp="200" kv="20"/>
  </actuator>
  <sensor>
    <jointpos name="joint1_pos" joint="joint1"/>
    <jointpos name="joint2_pos" joint="joint2"/>
    <jointpos name="joint3_pos" joint="joint3"/>
    <jointpos name="joint4_pos" joint="joint4"/>
    <jointpos name="joint5_pos" joint="joint5"/>
    <jointvel name="joint1_vel" joint="joint1"/>
    <jointvel name="joint2_vel" joint="joint2"/>
    <jointvel name="joint3_vel" joint="joint3"/>
    <jointvel name="joint4_vel" joint="joint4"/>
    <jointvel name="joint5_vel" joint="joint5"/>
    <jointactuatorfrc name="joint1_torque" joint="joint1"/>
    <jointactuatorfrc name="joint2_torque" joint="joint2"/>
    <jointactuatorfrc name="joint3_torque" joint="joint3"/>
    <jointactuatorfrc name="joint4_torque" joint="joint4"/>
    <jointactuatorfrc name="joint5_torque" joint="joint5"/>
  </sensor>
</mujoco>
"""

OSCILLATOR_XML = """\
<mujoco>
  <compiler autolimits="true"/>
  <option>
    <flag gravity="disable" contact="disable" limit="disable"/>
  </option>
  <worldbody>
    <body name="mass">
      <joint name="j1" axis="1 0 0" type="slide" range="0 0.5"
             stiffness="10" damping="1"/>
      <geom type="box" size=".1 .1 .1" mass="0.1"/>
    </body>
  </worldbody>
</mujoco>
"""


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def box_spec() -> mujoco.MjSpec:
  return mujoco.MjSpec.from_string(BOX_XML)


@pytest.fixture
def box_model(box_spec) -> mujoco.MjModel:
  return box_spec.compile()


@pytest.fixture
def arm_spec() -> mujoco.MjSpec:
  """Minimal 5-joint arm with sensors, actuators, textures/materials."""
  return mujoco.MjSpec.from_string(ARM_XML)


@pytest.fixture
def arm_model(arm_spec) -> mujoco.MjSpec:
  return arm_spec.compile()


@pytest.fixture
def oscillator_spec() -> mujoco.MjSpec:
  """Single-body oscillator with implicit (geom-based) inertia."""
  return mujoco.MjSpec.from_string(OSCILLATOR_XML)


@pytest.fixture
def simple_timeseries() -> timeseries.TimeSeries:
  """A TimeSeries with 5 data points and 2 columns: y = [x^2, 2*x^2]."""
  times = np.array([0.0, 1.0, 2.0, 3.0, 4.0])
  data = np.array([
      [0.0, 0.0],
      [1.0, 2.0],
      [4.0, 8.0],
      [9.0, 18.0],
      [16.0, 32.0],
  ])
  return timeseries.TimeSeries(times=times, data=data)


@pytest.fixture
def box_params(box_spec) -> parameter.ParameterDict:
  """ParameterDict for box model with modifier callbacks."""
  del box_spec
  pdict = parameter.ParameterDict()
  pdict.add(
      parameter.Parameter(
          "box_mass",
          [5],
          min_value=[4.5],
          max_value=[5.5],
          modifier=lambda s, p: setattr(
              _infer_inertial(s, "box"), "mass", p.value[0]
          ),
      )
  )
  pdict.add(
      parameter.Parameter(
          "solref1",
          [0.01],
          min_value=[0.002],
          max_value=[0.02],
          modifier=lambda s, p: s.pair("box_floor").solref.__setitem__(
              0, p.value[0]
          ),
      )
  )
  pdict.add(
      parameter.Parameter(
          "friction2",
          [0.005],
          min_value=[0],
          max_value=[0.01],
          modifier=lambda s, p: s.pair("box_floor").friction.__setitem__(
              1, p.value[0]
          ),
      )
  )
  return pdict
