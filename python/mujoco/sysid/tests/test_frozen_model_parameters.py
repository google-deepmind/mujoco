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
"""Fixed parameters still contribute their values to the physical model."""

import numpy as np
import pytest

import mujoco
from mujoco import sysid
import mujoco.rollout as rollout
from mujoco.sysid._src import model_modifier


@pytest.fixture
def frozen_mass_problem():
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
  mass = sysid.Parameter(
      "mass",
      1.0,
      0.5,
      3.0,
      frozen=True,
      modifier=lambda s, p: setattr(s.body("body"), "mass", p.value[0]),
  )
  mass.value[:] = 2.0
  gain = sysid.Parameter(
      "gain",
      1.0,
      0.25,
      2.0,
      modifier=lambda s, p: s.actuator("motor").gainprm.__setitem__(
          0, p.value[0]
      ),
  )
  return spec, sysid.ParameterDict({"mass": mass, "gain": gain})


@pytest.mark.parametrize("compile_model", [False, True])
def test_frozen_parameter_reaches_model(frozen_mass_problem, compile_model):
  spec, params = frozen_mass_problem
  params.update_from_vector(np.array([1.5]))

  if compile_model:
    model = model_modifier.apply_param_modifiers(params, spec)
  else:
    model = model_modifier.apply_param_modifiers_spec(params, spec).compile()

  np.testing.assert_allclose(model.body("body").mass, [2.0])
  np.testing.assert_allclose(model.actuator("motor").gainprm[0], 1.5)
  np.testing.assert_array_equal(params.as_vector(), [1.5])
  np.testing.assert_array_equal(params["mass"].nominal, [1.0])
  np.testing.assert_array_equal(params["mass"].value, [2.0])


def test_identification_retains_frozen_mass(frozen_mass_problem):
  spec, params = frozen_mass_problem
  true_spec = spec.copy()
  true_spec.body("body").mass = 2.0
  true_spec.actuator("motor").gainprm[0] = 1.5
  model = true_spec.compile()
  initial_state = sysid.create_initial_state(
      model, np.array([0.2]), np.array([0.1])
  )
  times = np.arange(21) * model.opt.timestep
  control = np.full((len(times), 1), 2.0)
  state, sensor = rollout.rollout(
      model, mujoco.MjData(model), initial_state, control[:-1]
  )
  sequence = sysid.ModelSequences(
      "body",
      spec,
      "measurement",
      initial_state,
      sysid.TimeSeries(times, control),
      sysid.TimeSeries.from_names(state[0, :, 0], sensor[0], model),
  )
  residual_fn = sysid.build_residual_fn(
      models_sequences=[sequence], n_threads=1
  )

  optimized, result = sysid.optimize(
      params, residual_fn, optimizer="mujoco", verbose=False, max_iters=30
  )

  np.testing.assert_allclose(optimized["gain"].value, [1.5], atol=1e-5)
  np.testing.assert_array_equal(optimized["mass"].value, [2.0])
  assert optimized["mass"].frozen
  assert result.x.size == 1
