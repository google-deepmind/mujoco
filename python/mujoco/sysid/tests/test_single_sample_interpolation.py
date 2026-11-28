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
"""Single-sample linear interpolation holds the one known observation."""

import numpy as np
import pytest

import mujoco
from mujoco import sysid
import mujoco.rollout as rollout


@pytest.mark.parametrize("sample_shape", [(), (1,), (3,), (2, 3)])
def test_single_sample_linear_interpolation(sample_shape):
  value = np.full(sample_shape, 2.5)
  series = sysid.TimeSeries(np.array([0.5]), value[None])
  query = np.array([[0.0, 0.5], [1.0, 0.5]])

  actual = series.interpolate(query)
  _, scalar = series.get(0.5)
  resampled = series.resample(new_times=np.array([0.0, 0.5, 1.0]))

  np.testing.assert_array_equal(
      actual, np.full(query.shape + sample_shape, 2.5)
  )
  np.testing.assert_array_equal(scalar, value)
  np.testing.assert_array_equal(
      resampled.data, np.full((3,) + sample_shape, 2.5)
  )
  actual[...] = 9.0
  np.testing.assert_array_equal(series.data, value[None])


def test_single_sample_does_not_bypass_interpolation_method_validation():
  series = sysid.TimeSeries(np.array([0.5]), np.array([[2.5]]))

  with pytest.raises(ValueError):
    series.interpolate(0.5, method="cubic")
  with pytest.raises(NotImplementedError):
    series.interpolate(0.5, method="invalid")
  np.testing.assert_array_equal(series.interpolate(0.5, method="zoh"), [[2.5]])


def test_single_observation_native_motor_gain_calibration():
  spec = mujoco.MjSpec.from_string("""
  <mujoco>
    <option timestep="0.01" gravity="0 0 0"/>
    <worldbody>
      <body>
        <joint name="slide" type="slide"/>
        <geom size="0.1"/>
      </body>
    </worldbody>
    <actuator><motor name="motor" joint="slide"/></actuator>
    <sensor><jointactuatorfrc name="force" joint="slide"/></sensor>
  </mujoco>
  """)
  model = spec.compile()
  initial_state = sysid.create_initial_state(model, np.array([0.2]))
  control = sysid.TimeSeries(np.array([0.0, 0.01]), np.full((2, 1), 2.0))
  state, sensor = rollout.rollout(
      model, mujoco.MjData(model), initial_state, control.data[:-1]
  )
  sequence = sysid.ModelSequences(
      "motor",
      spec,
      "measurement",
      initial_state,
      control,
      sysid.TimeSeries.from_names(state[0, :, 0], sensor[0], model),
  )
  params = sysid.ParameterDict()
  params.add(
      sysid.Parameter(
          "gain",
          1.5,
          0.5,
          2.0,
          modifier=lambda s, p: s.actuator("motor").gainprm.__setitem__(
              0, p.value[0]
          ),
      )
  )
  residual_fn = sysid.build_residual_fn(
      models_sequences=[sequence], n_threads=1
  )

  optimized, result = sysid.optimize(
      params, residual_fn, optimizer="scipy", verbose=False, max_iters=20
  )

  np.testing.assert_allclose(optimized["gain"].value, [1.0], atol=1e-6)
  assert np.isfinite(result.fun).all()
