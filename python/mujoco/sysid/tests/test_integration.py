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
"""End-to-end integration test using the box model."""

import pathlib
import tempfile

import mujoco
import mujoco.rollout as mj_rollout
from mujoco.sysid._src import signal_modifier
from mujoco.sysid._src import timeseries
from mujoco.sysid._src.io import save_results
from mujoco.sysid._src.model_modifier import _infer_inertial
from mujoco.sysid._src.optimize import optimize
from mujoco.sysid._src.parameter import Parameter
from mujoco.sysid._src.parameter import ParameterDict
from mujoco.sysid._src.residual import build_residual_fn
from mujoco.sysid._src.trajectory import ModelSequences
from mujoco.sysid._src.trajectory import create_initial_state
from mujoco.sysid.tests.conftest import BOX_XML
import numpy as np


def _generate_box_data(
    spec: mujoco.MjSpec,
    duration: float = 1.0,
) -> tuple[timeseries.TimeSeries, timeseries.TimeSeries, np.ndarray]:
  """Generate synthetic box-pushing data via rollout."""
  model = spec.compile()
  data = mujoco.MjData(model)

  n_steps = int(duration / model.opt.timestep)
  t = np.arange(n_steps) * model.opt.timestep
  force = (np.sin(t) * 3.0).reshape(-1, 1)
  control_ts = timeseries.TimeSeries(t, force)

  initial_state = create_initial_state(model, data.qpos, data.qvel, data.act)

  control_applied = force[:-1]
  state, _ = mj_rollout.rollout(model, data, initial_state, control_applied)
  state = np.squeeze(state, axis=0)

  sensor_ids = [1, 8]
  signal_mapping = {
      "pos_x": (timeseries.SignalType.MjStateQPos, np.array([0])),
      "vel_x": (timeseries.SignalType.MjStateQVel, np.array([1])),
  }
  state_times = state[:, 0]
  sensordata = timeseries.TimeSeries(
      state_times,
      state[:, sensor_ids],
      signal_mapping,
  )

  return control_ts, sensordata, initial_state


def _build_box_params() -> ParameterDict:
  """Build parameter dict with modifier callbacks matching box config."""
  pdict = ParameterDict()

  pdict.add(
      Parameter(
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
      Parameter(
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
      Parameter(
          "solref2",
          [1.0],
          min_value=[0.3],
          max_value=[1.7],
          frozen=True,
          modifier=lambda s, p: s.pair("box_floor").solref.__setitem__(
              1, p.value[0]
          ),
      )
  )
  pdict.add(
      Parameter(
          "friction1",
          [1.6],
          min_value=[0],
          max_value=[3.0],
          frozen=True,
          modifier=lambda s, p: s.pair("box_floor").friction.__setitem__(
              0, p.value[0]
          ),
      )
  )
  pdict.add(
      Parameter(
          "friction2",
          [0.005],
          min_value=[0],
          max_value=[0.01],
          modifier=lambda s, p: s.pair("box_floor").friction.__setitem__(
              1, p.value[0]
          ),
      )
  )
  pdict.add(
      Parameter(
          "friction3",
          [0.0001],
          min_value=[0],
          max_value=[0.001],
          frozen=True,
          modifier=lambda s, p: s.pair("box_floor").friction.__setitem__(
              2, p.value[0]
          ),
      )
  )
  return pdict


def test_box_end_to_end():
  """Full 5-stage pipeline: generate data, build residual, optimize 3 iters, save."""
  spec = mujoco.MjSpec.from_string(BOX_XML)

  # 1. Generate synthetic ground-truth data.
  control, sensordata, initial_state = _generate_box_data(spec, duration=1.0)

  # 2. Build config with known parameters.
  params = _build_box_params()

  # 3. Create ModelSequences.
  models_sequences = [
      ModelSequences(
          "box",
          spec,
          "push",
          initial_state,
          control,
          sensordata,
          allow_missing_sensors=True,
      )
  ]

  # 4. Define modify_residual (box uses state-based residual).
  def modify_residual(
      _params,
      _sensordata_predicted,
      sensordata_measured,
      model,
      _return_pred_all,
      state=None,
      **_kwargs,
  ):
    assert state is not None
    sensor_ids = [1, 8]
    statedata_predicted = timeseries.TimeSeries(
        state[:, 0],
        state[..., sensor_ids],
        {
            "pos_x": (timeseries.SignalType.MjStateQPos, np.array([0])),
            "vel_x": (timeseries.SignalType.MjStateQVel, np.array([1])),
        },
    )
    sensordata_measured = signal_modifier.apply_delayed_ts_window(
        sensordata_measured, statedata_predicted, 0.0, 0.0
    )
    statedata_predicted = statedata_predicted.resample(
        sensordata_measured.times
    )
    res = signal_modifier.weighted_diff(
        predicted_data=statedata_predicted.data,
        measured_data=sensordata_measured.data,
        model=model,
    )
    res = signal_modifier.normalize_residual(res, sensordata_measured.data)
    return res, statedata_predicted, sensordata_measured

  residual_fn = build_residual_fn(
      models_sequences=models_sequences,
      modify_residual=modify_residual,
  )

  # 5. Perturb params and optimize (3 iters to verify).
  rng = np.random.default_rng(42)
  params.randomize(rng=rng)

  # Compute initial cost.
  initial_residuals, _, _ = residual_fn(params.as_vector(), params)
  initial_cost = sum(np.sum(r**2) for r in initial_residuals)

  opt_params, opt_result = optimize(
      initial_params=params,
      residual_fn=residual_fn,
      optimizer="mujoco",
      max_iters=3,
      verbose=False,
  )

  # 6. Assert basic properties.
  assert opt_result.x.shape == params.as_vector().shape

  # Compute final cost.
  final_residuals, _, _ = residual_fn(opt_result.x, opt_params)
  final_cost = sum(np.sum(r**2) for r in final_residuals)
  assert (
      final_cost <= initial_cost
  ), f"Cost should decrease: {final_cost} > {initial_cost}"

  # 7. Save results to a temp dir.
  with tempfile.TemporaryDirectory() as tmpdir:
    save_results(
        experiment_results_folder=tmpdir,
        models_sequences=models_sequences,
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
    assert (result_dir / "box.xml").exists()
