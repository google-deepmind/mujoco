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
"""Broadcast rollout inputs retain the corresponding trajectory metadata."""

import numpy as np
import pytest

import mujoco
from mujoco.sysid._src import timeseries
from mujoco.sysid._src import trajectory


@pytest.mark.parametrize(
    ("model_input", "state_input", "control_input"),
    [
        ("single", "vector", "single"),
        ("single", "batched", "batched"),
        ("single", "vector", "batched"),
        ("single", "singleton", "batched"),
        ("single", "batched", "single"),
        ("single", "batched", "singleton"),
        ("singleton", "batched", "batched"),
        ("batched", "vector", "single"),
        ("batched", "batched", "batched"),
    ],
)
def test_sysid_rollout_broadcasts_inputs(
    model_input, state_input, control_input
):
  models = [mujoco.MjModel.from_xml_string(f"""
      <mujoco>
        <option timestep="0.01" gravity="0 0 0"/>
        <worldbody>
          <body>
            <joint name="slide" type="slide" axis="1 0 0"/>
            <geom size="0.1" mass="{mass}"/>
          </body>
        </worldbody>
        <actuator><motor name="motor" joint="slide"/></actuator>
        <sensor><jointpos name="position" joint="slide"/></sensor>
      </mujoco>
      """) for mass in (1.0, 2.0)]
  initial_states = np.stack([
      trajectory.create_initial_state(models[0], np.array([position]))
      for position in (0.1, 0.2)
  ])
  times = np.arange(5) * models[0].opt.timestep
  controls = [
      timeseries.TimeSeries(times, np.full((len(times), 1), force))
      for force in (0.5, 1.0)
  ]
  model_arg = {
      "single": models[0],
      "singleton": models[:1],
      "batched": models,
  }[model_input]
  state_arg = {
      "vector": initial_states[0],
      "singleton": initial_states[:1],
      "batched": initial_states,
  }[state_input]
  control_arg = {
      "single": controls[0],
      "singleton": controls[:1],
      "batched": controls,
  }[control_input]

  actual = trajectory.sysid_rollout(
      model_arg, mujoco.MjData(models[0]), control_arg, state_arg
  )

  nbatch = 2 if "batched" in (model_input, state_input, control_input) else 1
  assert len(actual) == nbatch
  for i, actual_traj in enumerate(actual):
    model = models[i if model_input == "batched" else 0]
    initial_state = initial_states[i if state_input == "batched" else 0]
    control = controls[i if control_input == "batched" else 0]
    expected = trajectory.sysid_rollout(
        model, mujoco.MjData(model), control, initial_state
    )[0]

    assert actual_traj.model is model
    np.testing.assert_array_equal(actual_traj.initial_state, initial_state)
    np.testing.assert_array_equal(actual_traj.control.times, control.times[:-1])
    np.testing.assert_array_equal(actual_traj.control.data, control.data[:-1])
    np.testing.assert_array_equal(actual_traj.state.data, expected.state.data)
    np.testing.assert_array_equal(
        actual_traj.sensordata.data, expected.sensordata.data
    )
