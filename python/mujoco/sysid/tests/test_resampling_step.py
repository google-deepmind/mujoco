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
"""Control resampling uses the timestep actually executed by the simulator."""

import numpy as np
import pytest

import mujoco
from mujoco import sysid
import mujoco.rollout as rollout


@pytest.mark.parametrize(
    "start,end,step,n_intervals",
    [
        (0.0, 0.35, 0.1, 4),
        (3.0, 3.35, 0.1, 4),
        (0.0, 0.3, 0.1, 3),
        (0.3, 0.9, 0.1, 6),
        (0.0, 0.1 + 0.2, 0.1, 3),
    ],
)
def test_resample_preserves_requested_step(start, end, step, n_intervals):
  times = np.array([start, end])
  series = sysid.TimeSeries(times, (2 * (times - start) + 3)[:, None])
  expected_times = start + step * np.arange(n_intervals + 1)

  actual = series.resample(target_dt=step)
  expected = series.resample(new_times=expected_times)

  np.testing.assert_allclose(actual.times, expected_times)
  np.testing.assert_allclose(np.diff(actual.times), step)
  np.testing.assert_allclose(actual.data, expected.data)
  assert actual.times[-1] >= end
  assert actual.times[-2] < end


@pytest.fixture
def off_grid_control_problem():
  spec = mujoco.MjSpec.from_string("""
  <mujoco>
    <option timestep="0.01" gravity="0 0 0"/>
    <worldbody>
      <body name="body">
        <joint name="slide" type="slide" axis="1 0 0"/>
        <inertial mass="1" pos="0 0 0" diaginertia="0.01 0.01 0.01"/>
        <geom size="0.1"/>
      </body>
    </worldbody>
    <actuator><motor name="motor" joint="slide"/></actuator>
    <sensor>
      <jointpos name="position" joint="slide"/>
      <jointvel name="velocity" joint="slide"/>
    </sensor>
  </mujoco>
  """)
  model = spec.compile()
  initial_state = sysid.create_initial_state(
      model, np.array([0.2]), np.array([0.1])
  )
  recorded_times = np.array([0.0, 0.02, 0.04, 0.06, 0.08, 0.095])
  control = sysid.TimeSeries(recorded_times, (100 * recorded_times)[:, None])
  simulation_times = np.arange(11) * model.opt.timestep
  simulation_control = control.resample(new_times=simulation_times).data
  state, sensor = rollout.rollout(
      model, mujoco.MjData(model), initial_state, simulation_control[:-1]
  )
  sequence = sysid.ModelSequences(
      "body",
      spec,
      "measurement",
      initial_state,
      control,
      sysid.TimeSeries.from_names(state[0, :, 0], sensor[0], model),
  )
  params = sysid.ParameterDict()
  params.add(
      sysid.Parameter(
          "mass",
          1.0,
          0.5,
          2.0,
          modifier=lambda s, p: setattr(s.body("body"), "mass", p.value[0]),
      )
  )
  residual_fn = sysid.build_residual_fn(
      models_sequences=[sequence], n_threads=1
  )
  return params, residual_fn


def test_native_residual_matches_off_grid_control_measurements(
    off_grid_control_problem,
):
  params, residual_fn = off_grid_control_problem

  values, _, _ = residual_fn(params.as_vector(), params)

  np.testing.assert_allclose(values[0], 0.0, atol=1e-12)


def test_optimization_recovers_mass_with_off_grid_controls(
    off_grid_control_problem,
):
  params, residual_fn = off_grid_control_problem
  params["mass"].value[:] = 1.2

  optimized, _ = sysid.optimize(
      params, residual_fn, optimizer="mujoco", verbose=False, max_iters=30
  )

  np.testing.assert_allclose(optimized["mass"].value, [1.0], atol=1e-5)
