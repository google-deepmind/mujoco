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
"""Residual closures support per-call options used by optimization reports."""

import numpy as np
import pytest

import mujoco
from mujoco import sysid


@pytest.fixture
def gain_problem():
  spec = mujoco.MjSpec.from_string("""
  <mujoco>
    <option timestep="0.01" gravity="0 0 0"/>
    <worldbody>
      <body name="mass">
        <joint name="slide" type="slide" axis="1 0 0"/>
        <geom size="0.1" mass="1"/>
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
  times = np.arange(6) * model.opt.timestep
  control = sysid.TimeSeries(times, np.full((len(times), 1), 0.5))
  rollout = sysid.sysid_rollout(
      model, mujoco.MjData(model), control, initial_state
  )[0]
  measured = rollout.sensordata.data.copy()
  measured[:, 0] *= 1.5
  sequence = sysid.ModelSequences(
      "gain_model",
      spec,
      "measurement",
      initial_state,
      control,
      sysid.TimeSeries(
          rollout.sensordata.times,
          measured,
          rollout.sensordata.signal_mapping,
      ),
  )
  params = sysid.ParameterDict()
  params.add(sysid.Parameter("gain", 1.0, 0.5, 2.0))
  transform = sysid.SignalTransform(normalize=False)
  transform.gain("position", params["gain"], target="predicted")
  transform.enable_sensors(["position"])
  residual_fn = sysid.build_residual_fn(
      models_sequences=[sequence],
      modify_residual=transform.apply,
      n_threads=1,
      return_pred_all=False,
  )
  return sequence, params, residual_fn


def test_return_pred_all_override_does_not_change_captured_default(
    gain_problem,
):
  _, params, residual_fn = gain_problem
  x = params.as_vector()
  before = residual_fn(x, params)
  all_signals = residual_fn(x, params, return_pred_all=True)
  after = residual_fn(x, params)

  assert before[1][0][0].data.shape[1] == 1
  assert all_signals[1][0][0].data.shape[1] == 2
  assert after[1][0][0].data.shape[1] == 1
  np.testing.assert_array_equal(before[0], after[0])
  np.testing.assert_allclose(
      all_signals[1][0][0].data[:, 0], before[1][0][0].data[:, 0]
  )


def test_thread_count_override_preserves_native_residual(gain_problem):
  _, params, residual_fn = gain_problem
  x = params.as_vector()
  expected = residual_fn(x, params)
  actual = residual_fn(x, params, n_threads=2)

  np.testing.assert_array_equal(actual[0], expected[0])
  np.testing.assert_array_equal(actual[1][0][0].data, expected[1][0][0].data)


def test_save_results_can_request_all_signals(gain_problem, tmp_path):
  sequence, params, residual_fn = gain_problem
  opt_params, result = sysid.optimize(
      params, residual_fn, optimizer="scipy", verbose=False, max_iters=20
  )
  np.testing.assert_allclose(opt_params["gain"].value, [1.5], atol=1e-6)

  sysid.save_results(
      tmp_path, [sequence], params, opt_params, result, residual_fn
  )

  for name in (
      "params_x_0.yaml",
      "params_x_hat.yaml",
      "results.pkl",
      "confidence.pkl",
      "gain_model.xml",
  ):
    assert (tmp_path / name).is_file()
  assert residual_fn(result.x, opt_params)[1][0][0].data.shape[1] == 1
