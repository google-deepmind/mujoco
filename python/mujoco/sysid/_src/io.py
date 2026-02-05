"""I/O utilities for saving system identification results."""

import os
import pathlib
import pickle
from collections.abc import Sequence

import scipy.optimize as scipy_optimize
from absl import logging

from mujoco.sysid._src import parameter
from mujoco.sysid._src.optimize import calculate_intervals
from mujoco.sysid._src.trajectory import ModelSequences


def save_results(
  experiment_results_folder: str | os.PathLike,
  models_sequences: Sequence[ModelSequences],
  initial_params: parameter.ParameterDict,
  opt_params: parameter.ParameterDict,
  opt_result: scipy_optimize.OptimizeResult,
  residual_fn,
):
  experiment_results_folder = pathlib.Path(experiment_results_folder)
  if not experiment_results_folder.exists():
    experiment_results_folder.mkdir(parents=True, exist_ok=True)
  logging.info("Experiment results will be saved to %s", experiment_results_folder)

  initial_params.save_to_disk(experiment_results_folder / "params_x_0.yaml")
  opt_params.save_to_disk(experiment_results_folder / "params_x_hat.yaml")

  with open(os.path.join(experiment_results_folder, "results.pkl"), "wb") as handle:
    pickle.dump(opt_result, handle, protocol=pickle.HIGHEST_PROTOCOL)

  # TODO: these intervals should be part of the params object.
  residuals_star, _, _ = residual_fn(opt_result.x, opt_params, return_pred_all=True)
  covariance, intervals = calculate_intervals(residuals_star, opt_result.jac)
  with open(os.path.join(experiment_results_folder, "confidence.pkl"), "wb") as handle:
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

  # Log nominal compared to initial.
  x0 = initial_params.as_vector()
  x_nominal = initial_params.as_nominal_vector()
  logging.info(
    "Initial Parameters\n%s",
    initial_params.compare_parameters(x0, opt_result.x, measured_params=x_nominal),
  )
