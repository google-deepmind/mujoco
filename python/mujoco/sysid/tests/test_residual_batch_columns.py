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
"""Finite-difference columns contain every measured trajectory's residuals."""

import numpy as np
import pytest

import mujoco
from mujoco import sysid
import mujoco.rollout as rollout
from mujoco.sysid._src import model_modifier
from mujoco.sysid._src import residual


@pytest.fixture
def mass_problem():
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
  times = np.arange(16) * model.opt.timestep
  trajectories = []
  for i in range(3):
    initial_state = sysid.create_initial_state(
        model, np.array([0.2 + 0.1 * i]), np.array([0.1])
    )
    control = np.full((len(times), 1), 0.5 + i)
    state, sensor = rollout.rollout(
        model, mujoco.MjData(model), initial_state, control[:-1]
    )
    trajectories.append(
        sysid.SystemTrajectory(
            model,
            sysid.TimeSeries(times, control),
            sysid.TimeSeries.from_names(state[0, :, 0], sensor[0], model),
            initial_state,
            state=None,
        )
    )

  params = sysid.ParameterDict()
  params.add(
      sysid.Parameter(
          "mass",
          1.5,
          0.5,
          2.0,
          modifier=lambda s, p: setattr(s.body("body"), "mass", p.value[0]),
      )
  )

  def evaluate(x, p, n_chunks=2):
    return residual.model_residual(
        x,
        p,
        lambda values: model_modifier.apply_param_modifiers(values, spec),
        trajectories[:n_chunks],
        n_threads=1,
    )

  return params, evaluate


@pytest.mark.parametrize(
    "n_chunks,n_fd", [(1, 1), (1, 2), (2, 1), (2, 2), (3, 2)]
)
def test_batched_columns_match_separate_native_evaluations(
    mass_problem, n_chunks, n_fd
):
  params, evaluate = mass_problem
  candidates = np.linspace(0.8, 1.4, n_fd)[None, :]
  expected = np.column_stack(
      [evaluate(candidates[:, i], params, n_chunks)[0] for i in range(n_fd)]
  )

  actual = evaluate(candidates, params, n_chunks)[0]

  assert actual.shape == expected.shape
  np.testing.assert_allclose(actual, expected)


@pytest.mark.parametrize("optimizer", ["mujoco", "scipy_parallel_fd"])
def test_multitrajectory_optimization_recovers_mass(mass_problem, optimizer):
  params, evaluate = mass_problem

  def residual_fn(x, p):
    values, predicted, measured = evaluate(x, p)
    return [values], predicted, measured

  optimized, result = sysid.optimize(
      params, residual_fn, optimizer=optimizer, verbose=False, max_iters=30
  )

  np.testing.assert_allclose(optimized["mass"].value, [1.0], atol=1e-5)
  assert result.jac.shape[1] == 1
  assert np.isfinite(result.jac).all()
