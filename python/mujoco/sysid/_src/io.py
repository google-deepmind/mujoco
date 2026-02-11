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

"""I/O utilities for saving system identification results."""

from collections.abc import Sequence
import os
import pathlib
import pickle

from absl import logging
from mujoco.sysid._src import parameter
from mujoco.sysid._src.optimize import calculate_intervals
from mujoco.sysid._src.trajectory import ModelSequences
import scipy.optimize as scipy_optimize


def save_results(
    experiment_results_folder: str | os.PathLike[str],
    models_sequences: Sequence[ModelSequences],
    initial_params: parameter.ParameterDict,
    opt_params: parameter.ParameterDict,
    opt_result: scipy_optimize.OptimizeResult,
    residual_fn,
):
  """Save optimization results and confidence intervals to disk.

  Args:
    experiment_results_folder: Directory where results are written.
    models_sequences: Model/sequence groups; identified XMLs are saved here.
    initial_params: Parameters before optimization.
    opt_params: Parameters after optimization.
    opt_result: Scipy OptimizeResult from the optimizer.
    residual_fn: Residual function used to compute confidence intervals.
  """
  experiment_results_folder = pathlib.Path(experiment_results_folder)
  if not experiment_results_folder.exists():
    experiment_results_folder.mkdir(parents=True, exist_ok=True)
  logging.info(
      "Experiment results will be saved to %s", experiment_results_folder
  )

  initial_params.save_to_disk(experiment_results_folder / "params_x_0.yaml")
  opt_params.save_to_disk(experiment_results_folder / "params_x_hat.yaml")

  with open(
      os.path.join(experiment_results_folder, "results.pkl"), "wb"
  ) as handle:
    pickle.dump(opt_result, handle, protocol=pickle.HIGHEST_PROTOCOL)

  # TODO(b/0): these intervals should be part of the params object.
  residuals_star, _, _ = residual_fn(
      opt_result.x, opt_params, return_pred_all=True
  )
  covariance, intervals = calculate_intervals(residuals_star, opt_result.jac)
  with open(
      os.path.join(experiment_results_folder, "confidence.pkl"), "wb"
  ) as handle:
    pickle.dump(
        {"cov": covariance, "intervals": intervals},
        handle,
        protocol=pickle.HIGHEST_PROTOCOL,
    )

  # Dump identified models to disk.
  for model_sequences in models_sequences:
    model_sequences.spec.to_file(
        (experiment_results_folder / f"{model_sequences.name}.xml").as_posix()
    )
