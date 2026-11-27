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
"""Signal arithmetic preserves fractional results from integer recordings."""

import numpy as np
import pytest

import mujoco
from mujoco import sysid
import mujoco.rollout as rollout


@pytest.mark.parametrize("dtype", [np.int64, np.float64])
@pytest.mark.parametrize(
    "operation,value,expected",
    [
        (sysid.apply_gain, 0.5, [0.0, 0.5, 1.0]),
        (sysid.apply_bias, 0.25, [0.25, 1.25, 2.25]),
        (sysid.apply_delay, 0.5, [0.0, 0.5, 1.5]),
    ],
)
def test_signal_modifiers_preserve_fractional_values(
    dtype, operation, value, expected
):
  values = np.array([[0, 10], [1, 11], [2, 12]], dtype=dtype)
  series = sysid.TimeSeries.from_custom_map(
      np.arange(3, dtype=float), values, ["selected", "other"]
  )
  param = sysid.Parameter("value", value, 0.0, 2.0)

  actual = operation(series, "selected", param)

  np.testing.assert_allclose(actual.data[:, 0], expected)
  np.testing.assert_array_equal(actual.data[:, 1], values[:, 1])
  np.testing.assert_array_equal(series.data, values)


@pytest.mark.parametrize("dtype", [np.int64, np.float64])
def test_fused_gain_and_bias_match_reference(dtype):
  values = np.array([[0, 10], [1, 11], [2, 12]], dtype=dtype)
  series = sysid.TimeSeries.from_custom_map(
      np.arange(3, dtype=float), values, ["selected", "other"]
  )
  gain = sysid.Parameter("gain", 0.5, 0.0, 2.0)
  bias = sysid.Parameter("bias", 0.25, 0.0, 2.0)
  params = sysid.ParameterDict({"gain": gain, "bias": bias})
  transform = sysid.SignalTransform()
  transform.gain("selected", gain)
  transform.bias("selected", bias)

  actual = transform._apply_gains_biases(series, "measured", params)
  expected = transform._apply_gains_biases_reference(series, "measured", params)

  np.testing.assert_allclose(actual.data[:, 0], [0.25, 0.75, 1.25])
  np.testing.assert_array_equal(actual.data, expected.data)
  np.testing.assert_array_equal(actual.data[:, 1], values[:, 1])
  np.testing.assert_array_equal(series.data, values)


def test_native_calibration_with_integer_sensor_counts():
  spec = mujoco.MjSpec.from_string("""
  <mujoco>
    <option timestep="0.01" gravity="0 0 0"/>
    <worldbody>
      <body><joint name="slide" type="slide"/><geom size="0.1"/></body>
    </worldbody>
    <actuator><motor name="motor" joint="slide"/></actuator>
    <sensor><jointactuatorfrc name="force" joint="slide"/></sensor>
  </mujoco>
  """)
  model = spec.compile()
  initial_state = sysid.create_initial_state(model, np.array([0.2]))
  times = np.arange(4) * model.opt.timestep
  control = np.full((len(times), 1), 2.0)
  state, sensor = rollout.rollout(
      model, mujoco.MjData(model), initial_state, control[:-1]
  )
  counts = (sensor[0] * 2).astype(np.int64)
  sequence = sysid.ModelSequences(
      "motor",
      spec,
      "measurement",
      initial_state,
      sysid.TimeSeries(times, control),
      sysid.TimeSeries.from_names(state[0, :, 0], counts, model),
  )
  gain = sysid.Parameter("gain", 0.8, 0.25, 2.0)
  params = sysid.ParameterDict({"gain": gain})
  transform = sysid.SignalTransform(normalize=False)
  transform.gain("force", gain, target="measured")
  residual_fn = sysid.build_residual_fn(
      models_sequences=[sequence], modify_residual=transform.apply, n_threads=1
  )

  optimized, _ = sysid.optimize(
      params, residual_fn, optimizer="scipy", verbose=False, max_iters=20
  )

  np.testing.assert_allclose(optimized["gain"].value, [0.5], atol=1e-6)
  np.testing.assert_array_equal(sequence.sensordata[0].data, counts)
