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
"""Default report generation for system identification results."""

from collections.abc import Sequence
import pathlib

from mujoco.sysid._src import model_modifier
from mujoco.sysid._src import parameter
from mujoco.sysid._src.optimize import calculate_intervals
from mujoco.sysid._src.residual import BuildModelFn
from mujoco.sysid._src.trajectory import ModelSequences
from mujoco.sysid.report.builder import ReportBuilder
from mujoco.sysid.report.sections.covariance import Covariance
from mujoco.sysid.report.sections.optimization_trace import OptimizationTrace
from mujoco.sysid.report.sections.parameters import ParametersTable
from mujoco.sysid.report.sections.signals import SignalReport
import numpy as np
import scipy.optimize as scipy_optimize


def default_report(
    models_sequences: Sequence[ModelSequences],
    initial_params: parameter.ParameterDict,
    opt_params: parameter.ParameterDict,
    residual_fn,
    opt_result: scipy_optimize.OptimizeResult,
    title="SysID",
    save_path=None,
    build_model: BuildModelFn | None = model_modifier.apply_param_modifiers,
    generate_videos=True,
) -> ReportBuilder:
  """Returns a ReportBuilder containing experiment results.

  Users needing a custom report can copy and modify this code.
  """
  from mujoco.sysid.report.sections.group import GroupSection
  from mujoco.sysid.report.sections.insights import AutomatedInsights
  from mujoco.sysid.report.sections.parameter_distribution import ParameterDistribution
  from mujoco.sysid.report.sections.row import RowSection
  from mujoco.sysid.report.sections.video import generate_video_from_trajectories
  from mujoco.sysid.report.sections.video import VideoPlayer

  ####################################
  # Build report
  # Sections:
  # Fit
  # Parameter tables
  # Confidence intervals
  # Extras: Optimization trace
  ####################################
  rb = ReportBuilder(title)

  if generate_videos:
    # 1. Video Player
    if save_path is None:
      raise ValueError("save_path is required when generate_videos=True")
    if build_model is None:
      raise ValueError("build_model is required when generate_videos=True")

    # Collect ALL trajectories from all model sequences
    all_trajectories = []
    for model_sequences in models_sequences:
      for traj in model_sequences.measured_rollout:
        all_trajectories.append(traj)

    # Use first model's spec for rendering
    model_spec_to_render = models_sequences[0].spec

    video_dir = pathlib.Path(save_path)
    video_dir.mkdir(parents=True, exist_ok=True)

    # Video 1: All (Initial + Nominal + Optimized)
    # all trajectories concatenated
    video_all_path = video_dir / "video_all.mp4"
    generate_video_from_trajectories(
        initial_params=initial_params,
        opt_params=opt_params,
        _build_model=build_model,
        trajectories=all_trajectories,
        model_spec=model_spec_to_render,
        output_filepath=video_all_path,
        fps=60,
    )

    # Video 2: Initial + Nominal (no optimized)
    video_init_path = video_dir / "video_init.mp4"
    generate_video_from_trajectories(
        initial_params=initial_params,
        opt_params=opt_params,
        _build_model=build_model,
        trajectories=all_trajectories,
        model_spec=model_spec_to_render,
        output_filepath=video_init_path,
        render_opt=False,
        fps=60,
    )

    # Video 3: Optimized + Nominal (no initial)
    video_opt_path = video_dir / "video_opt.mp4"
    generate_video_from_trajectories(
        initial_params=initial_params,
        opt_params=opt_params,
        _build_model=build_model,
        trajectories=all_trajectories,
        model_spec=model_spec_to_render,
        output_filepath=video_opt_path,
        render_initial=False,
        fps=60,
    )

    video_all_section = VideoPlayer(
        title="All Models",
        video_filepath=video_all_path,
        anchor="visual_run_all",
        autoplay=True,
        muted=True,
        width="100%",
        height=None,
        caption=(
            "<span class='color-initial'>Initial</span>, <span"
            " class='color-nominal'>Nominal</span>, <span"
            " class='color-optimized'>Optimized</span>"
        ),
    )

    video_init_section = VideoPlayer(
        title="Initial vs Nominal",
        video_filepath=video_init_path,
        anchor="visual_run_init",
        autoplay=True,
        muted=True,
        width="100%",
        height=None,
        caption=(
            "<span class='color-initial'>Initial</span>, <span"
            " class='color-nominal'>Nominal</span>"
        ),
    )

    video_opt_section = VideoPlayer(
        title="Optimized vs Nominal",
        video_filepath=video_opt_path,
        anchor="visual_run_opt",
        autoplay=True,
        muted=True,
        width="100%",
        height=None,
        caption=(
            "<span class='color-nominal'>Nominal</span>, <span"
            " class='color-optimized'>Optimized</span>"
        ),
    )

    rb.add_section(
        RowSection(
            title="Visual Comparison",
            sections=[video_all_section, video_init_section, video_opt_section],
            anchor="visual_comparison",
            description=(
                "Visual comparison of the system identification results. The"
                " nominal model is shown in green, the initial model in red,"
                " and the optimized model in blue."
            ),
        )
    )

  # 2. Automated Insights (Logs)
  rb.add_section(AutomatedInsights("Automated Insights", opt_params))

  # 3. Parameters Table (Unified)
  rb.add_section(
      ParametersTable(
          "Parameters", opt_params, initial_params, anchor="Parameters"
      )
  )

  # 4. Control Signals (per sequence, grouped like observations)
  # Get predictions for initial solution.
  names = [
      f"{model_sequences.name}\n{sequence}"
      for model_sequences in models_sequences
      for sequence in model_sequences.sequence_name
  ]
  _, pred0s, _ = residual_fn(
      initial_params.as_vector(), initial_params, return_pred_all=True
  )

  residuals_star, preds_star, records_star = residual_fn(
      opt_params.as_vector(), opt_params, return_pred_all=True
  )

  assert build_model is not None
  model_hat = build_model(initial_params, models_sequences[0].spec)

  # Build control signal reports for each sequence
  control_reports = []
  seq_idx = 0
  for model_sequences in models_sequences:
    for i, seq_name in enumerate(model_sequences.sequence_name):
      ctrl_ts = model_sequences.control[i]
      name = f"{model_sequences.name}\n{seq_name}"
      control_reports.append(
          SignalReport(
              f"Sequence: {name}",
              model_hat,
              title_prefix="",
              ts_dict={"control": ctrl_ts},
              collapsible=True,
          )
      )
      seq_idx += 1

  rb.add_section(
      GroupSection("Control Signals", control_reports, anchor="control_signals")
  )

  # 5. Observation Signals
  observation_reports = []
  for name, pred, record, pred0 in zip(
      names, preds_star, records_star, pred0s, strict=True
  ):
    obs_dict = {"initial": pred0[0], "nominal": record[0], "fitted": pred[0]}
    observation_reports.append(
        SignalReport(
            f"Sequence: {name}",
            model_hat,
            title_prefix="",
            ts_dict=obs_dict,
            collapsible=True,
        )
    )

  rb.add_section(
      GroupSection(
          "Observation Signals", observation_reports, anchor="observations"
      )
  )

  covariance, intervals = calculate_intervals(residuals_star, opt_result.jac)

  # 6. Parameter Distribution
  rb.add_section(
      ParameterDistribution(
          title="Parameter Distribution",
          opt_params=opt_params,
          initial_params=initial_params,
          confidence_intervals=intervals,
          anchor="param_dist",
      )
  )

  rb.add_section(
      Covariance(
          title="Covariance and Correlation",
          anchor="cov",
          covariance=covariance,
          parameter_dict=opt_params,
      )
  )

  # Add diagnostic optimization trace plots.
  if "extras" in opt_result:
    # Add to the report.
    rb.add_section(
        OptimizationTrace(
            title="Optimization Trace",
            anchor="opt",
            objective=opt_result.extras.get("objective"),
            candidate=opt_result.extras.get("candidate"),
            bounds=opt_params.get_bounds(),
            param_names=opt_params.get_non_frozen_parameter_names(),
        )
    )

  rb.build()
  if save_path:
    rb.save(save_path / "report.html")
  return rb
