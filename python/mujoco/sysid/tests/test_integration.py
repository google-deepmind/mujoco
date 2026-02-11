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
"""End-to-end integration tests for mujoco.sysid."""

import pathlib
import tempfile

import mujoco
import mujoco.rollout as rollout
from mujoco import sysid
import numpy as np


# ---------------------------------------------------------------------------
# Models
# ---------------------------------------------------------------------------

SPRING_MASS_XML = """\
<mujoco model="spring_mass">
  <option timestep="0.002">
    <flag contact="disable"/>
  </option>
  <worldbody>
    <body name="ball" pos="0 0 0.1">
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.001 0.001 0.001"/>
      <joint name="slide" type="slide" axis="1 0 0"
             stiffness="100" damping="5.0"/>
      <geom type="sphere" size="0.05"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="push" joint="slide"/>
  </actuator>
  <sensor>
    <jointpos name="position" joint="slide"/>
    <jointvel name="velocity" joint="slide"/>
  </sensor>
</mujoco>
"""

ARM_XML = """\
<mujoco model="arm">
  <compiler angle="radian" autolimits="true"/>
  <option integrator="implicitfast" timestep="0.002">
    <flag contact="disable"/>
  </option>
  <worldbody>
    <body name="link1" pos="0 0 0.1">
      <inertial pos="0 0 0.05" mass="1.0" diaginertia="0.01 0.01 0.005"/>
      <joint name="joint1" type="hinge" axis="0 0 1" range="-3.14 3.14"
             armature="0.5" damping="1.0"/>
      <geom type="capsule" fromto="0 0 0 0 0 0.1" size="0.04"/>
      <body name="link2" pos="0 0 0.1">
        <inertial pos="0 0 0.05" mass="0.8" diaginertia="0.008 0.008 0.004"/>
        <joint name="joint2" type="hinge" axis="0 1 0" range="-3.14 3.14"
               armature="0.4" damping="0.8"/>
        <geom type="capsule" fromto="0 0 0 0 0 0.1" size="0.035"/>
        <body name="link3" pos="0 0 0.1">
          <inertial pos="0 0 0.05" mass="0.6" diaginertia="0.006 0.006 0.003"/>
          <joint name="joint3" type="hinge" axis="0 1 0" range="-3.14 3.14"
                 armature="0.3" damping="0.6"/>
          <geom type="capsule" fromto="0 0 0 0 0 0.1" size="0.03"/>
          <body name="link4" pos="0 0 0.1">
            <inertial pos="0 0 0.04" mass="0.4" diaginertia="0.004 0.004 0.002"/>
            <joint name="joint4" type="hinge" axis="0 0 1" range="-3.14 3.14"
                   armature="0.2" damping="0.4"/>
            <geom type="capsule" fromto="0 0 0 0 0 0.08" size="0.025"/>
            <body name="link5" pos="0 0 0.08">
              <inertial pos="0 0 0.03" mass="0.2" diaginertia="0.002 0.002 0.001"/>
              <joint name="joint5" type="hinge" axis="0 1 0" range="-3.14 3.14"
                     armature="0.1" damping="0.2"/>
              <geom type="capsule" fromto="0 0 0 0 0 0.06" size="0.02"/>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="act1" joint="joint1"/>
    <motor name="act2" joint="joint2"/>
    <motor name="act3" joint="joint3"/>
    <motor name="act4" joint="joint4"/>
    <motor name="act5" joint="joint5"/>
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
  </sensor>
</mujoco>
"""

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5"]
TRUE_ARMATURE = {"joint1": 0.5, "joint2": 0.4, "joint3": 0.3,
                 "joint4": 0.2, "joint5": 0.1}


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _generate_data(xml, ctrl_fn, duration):
  """Rollout a model and return (spec, initial_state, control_ts, sensor_ts)."""
  spec = mujoco.MjSpec.from_string(xml)
  model = spec.compile()
  data = mujoco.MjData(model)

  n_steps = int(duration / model.opt.timestep)
  t = np.arange(n_steps) * model.opt.timestep
  ctrl = ctrl_fn(t)

  initial_state = sysid.create_initial_state(
      model, data.qpos, data.qvel, data.act
  )
  state, sensor = rollout.rollout(model, data, initial_state, ctrl[:-1])
  state = np.squeeze(state, axis=0)
  sensor = np.squeeze(sensor, axis=0)
  times = state[:, 0]

  control_ts = sysid.TimeSeries(t, ctrl)
  sensor_ts = sysid.TimeSeries.from_names(times, sensor, model)
  return spec, initial_state, control_ts, sensor_ts


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_spring_mass_recover_mass():
  """Recover true mass=1.0 starting from initial guess of 2.0."""
  spec, initial_state, control_ts, sensor_ts = _generate_data(
      SPRING_MASS_XML,
      ctrl_fn=lambda t: (
          5.0 * np.sin(2 * np.pi * 1.5 * t)
          + 3.0 * np.sin(2 * np.pi * 3.7 * t)
      ).reshape(-1, 1),
      duration=3.0,
  )

  params = sysid.ParameterDict()
  params.add(sysid.Parameter(
      "mass", nominal=1.0, min_value=0.3, max_value=3.0,
      modifier=lambda s, p: setattr(s.body("ball"), "mass", p.value[0]),
  ))
  params["mass"].value[:] = 2.0

  ms = sysid.ModelSequences(
      "spring_mass", spec, "measured", initial_state, control_ts, sensor_ts,
  )
  residual_fn = sysid.build_residual_fn(models_sequences=[ms])
  opt_params, _ = sysid.optimize(
      initial_params=params, residual_fn=residual_fn, optimizer="mujoco",
  )

  np.testing.assert_allclose(opt_params["mass"].value[0], 1.0, atol=1e-4)


def test_arm_recover_armature():
  """Recover 5 joint armature values and verify save_results."""
  spec, initial_state, control_ts, sensor_ts = _generate_data(
      ARM_XML,
      ctrl_fn=lambda t: np.column_stack([
          5.0 * np.sin(2 * np.pi * 0.5 * t),
          4.0 * np.sin(2 * np.pi * 0.7 * t + 0.5),
          3.0 * np.sin(2 * np.pi * 0.4 * t + 1.0),
          2.0 * np.sin(2 * np.pi * 0.9 * t + 1.5),
          1.0 * np.sin(2 * np.pi * 0.6 * t + 2.0),
      ]),
      duration=2.0,
  )

  params = sysid.ParameterDict()
  for name in JOINT_NAMES:
    params.add(sysid.Parameter(
        f"{name}_armature",
        nominal=TRUE_ARMATURE[name],
        min_value=0.001,
        max_value=1.0,
        modifier=lambda s, p, n=name: setattr(
            s.joint(n), "armature", p.value[0]
        ),
    ))
    params[f"{name}_armature"].value[:] = 0.01

  ms = sysid.ModelSequences(
      "arm", spec, "sinusoidal", initial_state, control_ts, sensor_ts,
  )
  residual_fn = sysid.build_residual_fn(models_sequences=[ms])
  opt_params, opt_result = sysid.optimize(
      initial_params=params, residual_fn=residual_fn, optimizer="mujoco",
  )

  for name in JOINT_NAMES:
    np.testing.assert_allclose(
        opt_params[f"{name}_armature"].value[0],
        TRUE_ARMATURE[name],
        atol=1e-4,
    )

  # Verify save_results produces expected files.
  with tempfile.TemporaryDirectory() as tmpdir:
    sysid.save_results(
        experiment_results_folder=tmpdir,
        models_sequences=[ms],
        initial_params=params,
        opt_params=opt_params,
        opt_result=opt_result,
        residual_fn=residual_fn,
    )
    result_dir = pathlib.Path(tmpdir)
    assert (result_dir / "params_x_0.yaml").exists()
    assert (result_dir / "params_x_hat.yaml").exists()
    assert (result_dir / "results.pkl").exists()
    assert (result_dir / "confidence.pkl").exists()
    assert (result_dir / "arm.xml").exists()
