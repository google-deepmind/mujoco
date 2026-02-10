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

"""Plotting utilities."""

from __future__ import annotations

from collections.abc import Sequence

from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import mujoco
from mujoco.sysid._src import parameter
import numpy as np


def plot_sensor_comparison(
    model: mujoco.MjModel,
    predicted_times: np.ndarray | None = None,
    predicted_data: np.ndarray | None = None,
    real_data: np.ndarray | None = None,
    real_times: np.ndarray | None = None,
    preid_data: np.ndarray | None = None,
    preid_times: np.ndarray | None = None,
    commanded_data: np.ndarray | None = None,
    commanded_times: np.ndarray | None = None,
    size_factor: float = 1.0,
    title_prefix: str = "",
    sensor_ids: list[int] | None = None,
):
  """Plots sensor trajectories from simulation and real data.

  Args:
      model: The model object providing sensor information.
      predicted_times: Optional 1D array of timestamps corresponding to
        simulation data.
      predicted_data: Optional 2D array of simulation sensor data with shape
        (num_timesteps, sensor_data_dimension).
      real_data: Optional 2D array of real sensor data with the same shape as
        predicted_data.
      real_times: A 1D array of timestamps corresponding to real data. If None
        and real_data is provided, the first available timestamp array is used.
      preid_data: Optional 2D array of pre-identification sensor data.
      preid_times: A 1D array of timestamps for pre-identification data.
      commanded_data: Optional 2D array of commanded sensor data.
      commanded_times: A 1D array of timestamps for commanded data.
      size_factor: A scaling factor for the figure size.
      title_prefix: Optional prefix for subplot titles.
      sensor_ids: Optional list of sensor indices to plot.
  """
  # Define a more appealing color palette
  predicted_color = "#1f77b4"  # Steel blue
  real_color = "#ff7f0e"  # Safety orange
  preid_color = "#2ca02c"  # Forest green
  commanded_color = "#9467bd"  # Purple

  # Determine the reference time array to use
  reference_times = None
  if predicted_times is not None:
    reference_times = predicted_times
  elif real_times is not None:
    reference_times = real_times
  elif preid_times is not None:
    reference_times = preid_times
  elif commanded_times is not None:
    reference_times = commanded_times
  else:
    raise ValueError("At least one time array must be provided")

  # Set times for data sources that don't have their own time arrays
  if real_data is not None and real_times is None:
    real_times = reference_times
  if preid_data is not None and preid_times is None:
    preid_times = reference_times
  if commanded_data is not None and commanded_times is None:
    commanded_times = reference_times
  if predicted_data is not None and predicted_times is None:
    predicted_times = reference_times

  if sensor_ids is None:
    sensor_ids = list(range(model.nsensor))
  assert predicted_data is not None
  n_plots = predicted_data.shape[1]

  fig, axes = plt.subplots(
      n_plots,
      1,
      figsize=(10 * size_factor, 2.5 * n_plots * size_factor),
      sharex=True,
  )
  if n_plots == 1:
    axes = [axes]
  axes = list(axes)  # pyright: ignore[reportArgumentType]

  # Set an overall title for the figure.
  fig.suptitle(title_prefix + " Sensors", fontsize=14)  # , y=1.02)

  # Loop over each sensor.
  plot_i = 0
  sensor_dim = 1
  j = 0
  dim_str = ""
  for sensor_id in sensor_ids:
    sensor = model.sensor(sensor_id)
    sensor_name = sensor.name
    sensor_dim = int(sensor.dim[0])
    sensor_addr = int(sensor.adr[0])

    for j in range(sensor_dim):
      ax = axes[plot_i]
      plot_i += 1
      dim_str = "" if sensor_dim == 1 else f" {j}"
      if predicted_data is not None:
        assert predicted_times is not None
        predicted_signal = predicted_data[
            :, sensor_addr : sensor_addr + sensor_dim
        ]
        ax.plot(
            predicted_times,
            predicted_signal[:, j],
            lw=2,
            color=predicted_color,
            alpha=0.8,
            label="Sim" + dim_str,
        )
      if real_data is not None:
        assert real_times is not None
        real_signal = real_data[:, sensor_addr : sensor_addr + sensor_dim]
        ax.plot(
            real_times,
            real_signal[:, j],
            lw=2,
            color=real_color,
            linestyle="--",
            alpha=0.7,
            label="Real" + dim_str,
        )
      if preid_data is not None:
        assert preid_times is not None
        preid_signal = preid_data[:, sensor_addr : sensor_addr + sensor_dim]
        ax.plot(
            preid_times,
            preid_signal[:, j],
            lw=2,
            color=preid_color,
            linestyle=":",
            alpha=0.6,
            label="Pre-ID" + dim_str,
        )
      if commanded_data is not None:
        assert commanded_times is not None
        commanded_signal = commanded_data[
            :, sensor_addr : sensor_addr + sensor_dim
        ]
        ax.plot(
            commanded_times,
            commanded_signal[:, j],
            lw=2,
            color=commanded_color,
            linestyle="-.",
            alpha=0.6,
            label="Commanded" + dim_str,
        )
      # Place the sensor name in a white box in the top-left corner.
      ax.text(
          0.02,
          0.9,
          sensor_name + dim_str,
          transform=ax.transAxes,
          fontsize=10,
          weight="bold",
          verticalalignment="top",
          horizontalalignment="left",
          bbox=dict(facecolor="white", alpha=0.8, edgecolor="none"),
      )

      # Enable a dashed grid.
      ax.grid(True, linestyle="--", alpha=0.7)

  # Loop over "extra" sensors from the user
  for _ in range(plot_i, n_plots):
    sensor_name = "user_sensor"
    dim_str = "" if sensor_dim == 1 else f" {j}"
    ax = axes[plot_i]
    plot_i += 1
    if predicted_data is not None:
      assert predicted_times is not None
      predicted_signal = predicted_data[:, plot_i - 1]
      ax.plot(
          predicted_times,
          predicted_signal,
          lw=2,
          color=predicted_color,
          alpha=0.8,
          label="Sim",
      )
    if real_data is not None:
      assert real_times is not None
      real_signal = real_data[:, plot_i - 1]
      ax.plot(
          real_times,
          real_signal,
          lw=2,
          color=real_color,
          linestyle="--",
          alpha=0.7,
          label="Real",
      )
    if preid_data is not None:
      assert preid_times is not None
      preid_signal = preid_data[:, plot_i - 1]
      ax.plot(
          preid_times,
          preid_signal,
          lw=2,
          color=preid_color,
          linestyle=":",
          alpha=0.6,
          label="Pre-ID",
      )
    if commanded_data is not None:
      assert commanded_times is not None
      commanded_signal = commanded_data[:, plot_i - 1]
      ax.plot(
          commanded_times,
          commanded_signal,
          lw=2,
          color=commanded_color,
          linestyle="-.",
          alpha=0.6,
          label="Commanded",
      )
    # Place the sensor name in a white box in the top-left corner.
    ax.text(
        0.02,
        0.9,
        sensor_name + dim_str,
        transform=ax.transAxes,
        fontsize=10,
        weight="bold",
        verticalalignment="top",
        horizontalalignment="left",
        bbox=dict(facecolor="white", alpha=0.8, edgecolor="none"),
    )

    # Enable a dashed grid.
    ax.grid(True, linestyle="--", alpha=0.7)

  # Add a unified, figure-level legend if any data is provided.
  legend_handles = []
  if predicted_data is not None:
    legend_handles.append(
        Line2D([0], [0], color=predicted_color, lw=2, label="Simulation")
    )
  if real_data is not None:
    legend_handles.append(
        Line2D([0], [0], color=real_color, lw=2, linestyle="--", label="Real")
    )
  if preid_data is not None:
    legend_handles.append(
        Line2D([0], [0], color=preid_color, lw=2, linestyle=":", label="Pre-ID")
    )
  if commanded_data is not None:
    legend_handles.append(
        Line2D(
            [0],
            [0],
            color=commanded_color,
            lw=2,
            linestyle="-.",
            label="Commanded",
        )
    )

  if legend_handles:
    fig.legend(
        handles=legend_handles,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.935),
        ncol=len(legend_handles),
        fancybox=True,
        shadow=True,
        fontsize=10,
        title="Data Source",
    )

  fig.supxlabel("Time (s)", fontsize=8)
  plt.tight_layout(rect=(0, 0.03, 1, 0.9))


def plot_objective(
    objective: Sequence[float],
    figsize: tuple[float, float] = (8, 5),
):
  """Plot the objective value over optimization iterations."""
  plt.figure(figsize=figsize)
  plt.plot(objective, linewidth=2, marker="o", markersize=4)
  final_value = objective[-1]
  if abs(final_value) < 1e-3 or abs(final_value) > 1e3:
    final_str = f"{final_value:.2e}"
  else:
    final_str = f"{final_value:.4f}"
  plt.title(f"Objective Over Time (Final: {final_str})", fontsize=14, pad=10)
  plt.grid(True, linestyle="--", alpha=0.6)
  plt.xlabel("Iteration", fontsize=12)
  plt.ylabel("Objective", fontsize=12)
  plt.xticks(fontsize=10)
  plt.yticks(fontsize=10)
  plt.tight_layout()


def plot_candidate(
    candidate: Sequence[np.ndarray],
    bounds: (
        tuple[Sequence[float] | np.ndarray, Sequence[float] | np.ndarray] | None
    ) = None,
    param_names: Sequence[str] | None = None,
    figsize: tuple[float, float] = (12, 2.5),
    dims_per_page: int = 6,
    log_diff: bool = True,
    bound_eps: float = 1e-3,
):
  """Plot candidate parameter values and their diffs over iterations."""
  values = np.array(candidate)  # shape: (n_iter, n_dim)
  n_iter, n_dim = values.shape
  diffs = np.diff(values, axis=0)

  mins = np.full(n_dim, -np.inf)
  maxs = np.full(n_dim, np.inf)
  if bounds is not None:
    mins = np.array(bounds[0])
    maxs = np.array(bounds[1])
    assert mins.shape == (n_dim,) and maxs.shape == (n_dim,)

  if param_names is not None:
    assert len(param_names) == n_dim

  # TODO(b/0) support pages, they are currently broken because
  # saving to disk overwrites the pages
  # n_pages = math.ceil(n_dim / dims_per_page)
  n_pages = 1
  for _page in range(n_pages):
    # start = page * dims_per_page
    # end = min((page + 1) * dims_per_page, n_dim)
    start = 0
    end = n_dim
    dims_in_page = end - start

    fig, axes = plt.subplots(
        dims_in_page,
        2,
        figsize=(figsize[0], figsize[1] * dims_in_page),
        sharex="col",
    )
    if dims_in_page == 1:
      axes = np.expand_dims(axes, 0)

    for i, dim in enumerate(range(start, end)):
      label = param_names[dim] if param_names is not None else f"Dim {dim}"
      ax_val, ax_diff = axes[i]

      vals = values[:, dim]
      ax_val.set_ylabel(label, fontsize=10)
      ax_val.grid(True, linestyle="--", alpha=0.6)
      ax_val.tick_params(labelsize=9)

      if bounds is not None:
        lower, upper = mins[dim], maxs[dim]
        ax_val.axhspan(lower, upper, color="gray", alpha=0.08)
        ax_val.plot(
            [0, n_iter - 1],
            [lower, lower],
            color="gray",
            linestyle="--",
            alpha=0.3,
            linewidth=1,
        )
        ax_val.plot(
            [0, n_iter - 1],
            [upper, upper],
            color="gray",
            linestyle="--",
            alpha=0.3,
            linewidth=1,
        )
        near_lower = np.abs(vals - lower) < bound_eps
        near_upper = np.abs(vals - upper) < bound_eps
        near_bound = near_lower | near_upper
        for t in range(1, n_iter):
          is_near_prev = near_bound[t - 1]
          is_near_curr = near_bound[t]
          color = "#d62728" if is_near_prev and is_near_curr else "#1f77b4"
          ax_val.plot(
              [t - 1, t], [vals[t - 1], vals[t]], color=color, linewidth=2
          )
          ax_val.plot(t, vals[t], marker="o", markersize=3, color=color)
        # Overlay triangle markers for near-bound points
        for t in range(n_iter):
          if near_lower[t]:
            ax_val.plot(t, vals[t], marker="v", markersize=6, color="#d62728")
          elif near_upper[t]:
            ax_val.plot(t, vals[t], marker="^", markersize=6, color="#d62728")
      else:
        ax_val.plot(vals, linewidth=2, marker="o", markersize=3)

      # Annotate final value
      final_val = vals[-1]
      final_str = (
          f"{final_val:.2e}"
          if abs(final_val) < 1e-3 or abs(final_val) > 1e3
          else f"{final_val:.4f}"
      )
      ax_val.text(
          n_iter - 1,
          final_val,
          final_str,
          ha="right",
          va="bottom",
          fontsize=9,
          color="blue",
      )

      # Annotate final value.
      final_val = values[-1, dim]
      final_str = (
          f"{final_val:.2e}"
          if abs(final_val) < 1e-3 or abs(final_val) > 1e3
          else f"{final_val:.4f}"
      )
      ax_val.text(
          n_iter - 1,
          final_val,
          final_str,
          ha="right",
          va="bottom",
          fontsize=9,
          color="blue",
      )

      # Plot diffs
      if log_diff:
        eps = 1e-12
        ax_diff.plot(
            np.log10(np.abs(diffs[:, dim]) + eps),
            linewidth=2,
            marker="x",
            markersize=4,
            color="tab:orange",
        )
        ax_diff.set_ylabel("log Δ", fontsize=9)
      else:
        ax_diff.plot(
            diffs[:, dim],
            linewidth=2,
            marker="x",
            markersize=4,
            color="tab:orange",
        )

      ax_diff.grid(True, linestyle="--", alpha=0.6)
      ax_diff.tick_params(labelsize=9)

    # Set common labels/titles
    axes[-1, 0].set_xlabel("Iteration", fontsize=12)
    axes[-1, 1].set_xlabel("Iteration", fontsize=12)
    axes[0, 0].set_title("Candidate Value", fontsize=12)
    axes[0, 1].set_title("Δ Candidate (Diff)", fontsize=12)

    fig.suptitle(
        f"Candidate Values and Changes (Dims {start}-{end - 1})", fontsize=14
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))


def plot_candidate_heatmap(
    candidate: Sequence[np.ndarray],
    param_names: Sequence[str] | None = None,
    bounds: (
        tuple[Sequence[float] | np.ndarray, Sequence[float] | np.ndarray] | None
    ) = None,
    normalize: bool = True,
    figsize: tuple[float, float] = (10, 6),
    cmap: str = "RdBu",
    show_colorbar: bool = True,
    bound_eps: float = 1e-3,
):
  """Plot a heatmap of candidate parameter values over iterations."""
  data = np.array(candidate).T  # shape: (n_dim, n_iter)
  n_dim = data.shape[0]

  if normalize and bounds is not None:
    min_bounds, max_bounds = bounds
    assert len(min_bounds) == len(max_bounds) == n_dim
    norm_data = np.empty_like(data)
    for i in range(n_dim):
      min_val = min_bounds[i]
      max_val = max_bounds[i]
      denom = max_val - min_val if max_val > min_val else 1.0
      norm_data[i] = (data[i] - min_val) / denom
  else:
    norm_data = data

  fig, ax = plt.subplots(figsize=figsize)
  im = ax.imshow(norm_data, aspect="auto", cmap=cmap)

  ax.set_xlabel("Iteration", fontsize=12)
  ax.set_ylabel("Parameter", fontsize=12)

  # Y-axis labels.
  if param_names is not None:
    assert len(param_names) == n_dim
    ax.set_yticks(np.arange(n_dim))
    ax.set_yticklabels(param_names, fontsize=10)
  else:
    ax.set_yticks(np.arange(n_dim))
    ax.set_yticklabels([f"Dim {i}" for i in range(n_dim)], fontsize=10)

  # Plot Xs where values are at bounds.
  if bounds is not None:
    min_bounds, max_bounds = bounds
    for dim in range(n_dim):
      min_val = min_bounds[dim]
      max_val = max_bounds[dim]
      for iter_idx, val in enumerate(data[dim]):
        if abs(val - min_val) < bound_eps or abs(val - max_val) < bound_eps:
          ax.plot(iter_idx, dim, "kx", markersize=6, markeredgewidth=1.5)

  if show_colorbar:
    cbar = fig.colorbar(im, ax=ax)
    label = "Normalized Value" if normalize else "Value"
    cbar.set_label(label, fontsize=12)

  ax.set_title("Candidate Heatmap", fontsize=14)
  fig.tight_layout()


def parameter_confidence(
    all_exp_names: Sequence[str],
    all_params: Sequence[parameter.ParameterDict],
    all_intervals: Sequence[np.ndarray],
    cols: int = 5,
    gt_params: parameter.ParameterDict | None = None,
):
  """Plot parameter estimates with confidence intervals."""
  named_estimates = {}
  # Create an entry for every non-frozen parameter
  for params in all_params:
    param_names = params.get_non_frozen_parameter_names()
    for name in param_names:
      if name not in named_estimates:
        named_estimates[name] = {
            "x": [],
            "intervals": [],
            "min_bounds": [],
            "max_bounds": [],
            "plot_labels": [],
        }

  for exp_name, params, intervals in zip(
      all_exp_names, all_params, all_intervals, strict=True
  ):
    param_names = params.get_non_frozen_parameter_names()
    xs = params.as_vector()
    bounds = params.get_bounds()
    assert xs.shape[0] == len(param_names)
    if gt_params is not None:
      for name in param_names:
        if name in gt_params:
          named_estimates[name]["xgt"] = gt_params[name].value[0]
        else:
          assert name[-1] == "]"
          left_bracket_i = name[::-1].find("[")
          index = int(name[-left_bracket_i:-1])
          named_estimates[name]["xgt"] = gt_params[
              name[: -left_bracket_i - 1]
          ].value[index]

    for i, (name, x, interval) in enumerate(
        zip(param_names, xs, intervals, strict=True)
    ):
      named_estimates[name]["x"].append(x)
      named_estimates[name]["intervals"].append(interval)
      named_estimates[name]["min_bounds"].append(bounds[0][i])
      named_estimates[name]["max_bounds"].append(bounds[1][i])
      named_estimates[name]["plot_labels"].append(exp_name)

  rows = len(named_estimates) // cols + 1
  fig, axs = plt.subplots(
      rows, cols, figsize=(20, 2 * (len(named_estimates) // cols + 1))
  )
  if rows == 1:
    axs = [axs]

  for i, name in enumerate(named_estimates):
    x_list = named_estimates[name]["x"]
    intervals = named_estimates[name]["intervals"]
    plot_labels = named_estimates[name]["plot_labels"]

    row = i % rows
    col = i // rows

    min_bound = np.min(named_estimates[name]["min_bounds"])
    max_bound = np.min(named_estimates[name]["max_bounds"])

    for j, (x, interval, plot_label) in enumerate(
        zip(x_list, intervals, plot_labels, strict=True)
    ):
      if not np.isfinite(interval) or 2.0 * interval > 2.0 * (
          max_bound - min_bound
      ):
        interval = 2.0 * (max_bound - min_bound)
        eb = axs[row][col].errorbar(x, -j, xerr=interval)
        eb[-1][0].set_linestyle("--")
      else:
        axs[row][col].errorbar(x, -j, xerr=interval)
      axs[row][col].scatter(x, -j, marker="x", label=plot_label)

    axs[row][col].set_xlim([min_bound, max_bound])
    axs[row][col].yaxis.set_ticklabels([])
    axs[row][col].set_title(name)
    axs[row][col].grid(True)
    axs[row][col].legend(
        fontsize=5, loc="upper right", bbox_to_anchor=(1.4, 1.0)
    )
    if gt_params is not None:
      axs[row][col].axvline(named_estimates[name]["xgt"], color="b", ls="--")

  fig.tight_layout()


def render_rollout(
    model: mujoco.MjModel | Sequence[mujoco.MjModel],
    data: mujoco.MjData,
    state: np.ndarray,
    framerate: int,
    camera: str | int = -1,
    width: int = 640,
    height: int = 480,
    light_pos: Sequence[float] | None = None,
) -> list[np.ndarray]:
  """Renders a rollout or batch of rollouts.

  Args:
    model: Single model or list of models (one per batch).
    data: MjData scratch object.
    state: State array of shape (nbatch, nsteps, nstate).
    framerate: Frames per second to render.
    camera: Camera name or ID.
    width: Image width.
    height: Image height.
    light_pos: Optional light position [x, y, z] to add a spotlight.

  Returns:
    List of rendered frames (numpy arrays).
  """
  nbatch = state.shape[0]

  if isinstance(model, mujoco.MjModel):
    models_list = [model] * nbatch
  else:
    models_list = list(model)
    if len(models_list) == 1:
      models_list = models_list * nbatch
    else:
      assert len(models_list) == nbatch

  # Visual options
  vopt = mujoco.MjvOption()
  vopt.geomgroup[3] = 1  # Show visualization geoms

  pert = mujoco.MjvPerturb()
  catmask = mujoco.mjtCatBit.mjCAT_DYNAMIC.value

  # Simulate and render.
  frames = []

  with mujoco.Renderer(models_list[0], height=height, width=width) as renderer:
    for i in range(state.shape[1]):
      # Check if we should capture this frame based on framerate
      if len(frames) < i * models_list[0].opt.timestep * framerate:
        for j in range(state.shape[0]):
          # Set state
          mujoco.mj_setState(
              models_list[j],
              data,
              state[j, i, :],
              mujoco.mjtState.mjSTATE_FULLPHYSICS.value,
          )
          mujoco.mj_forward(models_list[j], data)

          # Use first model to make the scene, add subsequent models
          if j == 0:
            renderer.update_scene(data, camera, scene_option=vopt)
          else:
            mujoco.mjv_addGeoms(
                models_list[j], data, vopt, pert, catmask, renderer.scene
            )

        # Add light, if requested
        if light_pos is not None:
          if renderer.scene.nlight < 100:  # check limit
            light = renderer.scene.lights[renderer.scene.nlight]
            light.ambient = [0, 0, 0]
            light.attenuation = [1, 0, 0]
            light.castshadow = 1
            light.cutoff = 45
            light.diffuse = [0.8, 0.8, 0.8]
            light.dir = [0, 0, -1]
            light.type = mujoco.mjtLightType.mjLIGHT_SPOT
            light.exponent = 10
            light.headlight = 0
            light.specular = [0.3, 0.3, 0.3]
            light.pos = light_pos
            renderer.scene.nlight += 1

        # Render and add the frame.
        pixels = renderer.render()
        frames.append(pixels)
  return frames
