// Copyright 2025 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "experimental/platform/sim/sim_profiler.h"

#include <algorithm>
#include <vector>

#include <imgui.h>
#include <implot.h>
#include <mujoco/mujoco.h>
#include "experimental/platform/ux/imgui_widgets.h"

namespace mujoco::platform {

SimProfiler::SimProfiler() { Clear(); }

void SimProfiler::Clear() {
  head_ = 0;
  num_frames_ = 0;
  cpu_total_.clear();
  cpu_collision_.clear();
  cpu_prepare_.clear();
  cpu_solve_.clear();
  cpu_other_.clear();
  dim_dof_.clear();
  dim_body_.clear();
  dim_constraint_.clear();
  dim_sqrt_nnz_.clear();
  dim_contact_.clear();
  dim_iteration_.clear();

  cpu_total_.resize(kProfilerMaxFrames, 0);
  cpu_collision_.resize(kProfilerMaxFrames, 0);
  cpu_prepare_.resize(kProfilerMaxFrames, 0);
  cpu_solve_.resize(kProfilerMaxFrames, 0);
  cpu_other_.resize(kProfilerMaxFrames, 0);
  dim_dof_.resize(kProfilerMaxFrames, 0);
  dim_body_.resize(kProfilerMaxFrames, 0);
  dim_constraint_.resize(kProfilerMaxFrames, 0);
  dim_sqrt_nnz_.resize(kProfilerMaxFrames, 0);
  dim_contact_.resize(kProfilerMaxFrames, 0);
  dim_iteration_.resize(kProfilerMaxFrames, 0);
}

void SimProfiler::Update(const mjModel* model, const mjData* data) {
  // CPU timers.
  mjtNum total = data->timer[mjTIMER_STEP].duration;
  mjtNum number = static_cast<mjtNum>(data->timer[mjTIMER_STEP].number);
  if (number == 0.0) {
    total = data->timer[mjTIMER_FORWARD].duration;
    number = static_cast<mjtNum>(data->timer[mjTIMER_FORWARD].number);
  }
  if (number == 0.0) {
    // This can happen if the simulation is paused.
    return;
  }

  mjtNum avg_total = total / number;
  cpu_total_[head_] = avg_total;

  mjtNum collision = data->timer[mjTIMER_POS_COLLISION].duration / number;
  cpu_collision_[head_] = collision;

  mjtNum prepare = (data->timer[mjTIMER_POS_MAKE].duration / number) +
                   (data->timer[mjTIMER_POS_PROJECT].duration / number);
  cpu_prepare_[head_] = prepare;

  mjtNum solve = data->timer[mjTIMER_CONSTRAINT].duration / number;
  cpu_solve_[head_] = solve;

  mjtNum other = avg_total - collision - prepare - solve;
  cpu_other_[head_] = other;

  // Solver diagnostics.
  mjtNum sqrt_nnz = 0;
  int solver_niter = 0;
  const int nisland =
      data->nefc ? mjMAX(1, mjMIN(data->nisland, mjNISLAND)) : 0;
  for (int island = 0; island < nisland; island++) {
    sqrt_nnz += data->solver_nnz[island];
    solver_niter += data->solver_niter[island];
  }
  sqrt_nnz = mju_sqrt(sqrt_nnz);

  int nv = (model->opt.enableflags & mjENBL_SLEEP) ? data->nv_awake : model->nv;
  dim_dof_[head_] = nv;

  int nbody = (model->opt.enableflags & mjENBL_SLEEP) ? data->nbody_awake
                                                      : model->nbody;
  dim_body_[head_] = nbody;

  dim_constraint_[head_] = data->nefc;
  dim_sqrt_nnz_[head_] = sqrt_nnz;
  dim_contact_[head_] = data->ncon;
  dim_iteration_[head_] = static_cast<float>(solver_niter) / mjMAX(1, nisland);

  head_ = (head_ + 1) % kProfilerMaxFrames;
  num_frames_ = std::min(num_frames_ + 1, kProfilerMaxFrames);
}

SimProfiler::Summary SimProfiler::GetSummary() const {
  Summary summary;
  summary.capacity = kProfilerMaxFrames;
  summary.num_frames = num_frames_;

  // Valid data is contiguous in [0, num_frames_): the ring buffer fills from
  // index 0 and num_frames_ saturates at the capacity, so once it wraps the
  // entire buffer is valid.
  const int n = num_frames_;
  auto stats = [n](const std::vector<float>& buffer) {
    MetricStats metric;
    if (n == 0) {
      return metric;
    }
    metric.min = metric.max = buffer[0];
    double sum = 0;
    for (int i = 0; i < n; ++i) {
      float value = buffer[i];
      sum += value;
      metric.min = std::min(metric.min, value);
      metric.max = std::max(metric.max, value);
    }
    metric.average = static_cast<float>(sum / n);
    return metric;
  };

  summary.cpu_total = stats(cpu_total_);
  summary.cpu_collision = stats(cpu_collision_);
  summary.cpu_prepare = stats(cpu_prepare_);
  summary.cpu_solve = stats(cpu_solve_);
  summary.cpu_other = stats(cpu_other_);
  summary.dim_dof = stats(dim_dof_);
  summary.dim_body = stats(dim_body_);
  summary.dim_constraint = stats(dim_constraint_);
  summary.dim_sqrt_nnz = stats(dim_sqrt_nnz_);
  summary.dim_contact = stats(dim_contact_);
  summary.dim_iteration = stats(dim_iteration_);
  return summary;
}

void SimProfiler::CpuTimeGraph(ImVec2 plot_size) {
  ImPlotFlags flags =
      ImPlot_SetupPlotFlags(plot_size) | ImPlotFlags_NoMouseText;
  if (ImPlot::BeginPlot("CPU msec vs frame", plot_size, flags)) {
    ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 2.0f);
    ImPlot_SetupTimeAxis(plot_size, "");
    ImPlot_SetupValueAxis(plot_size, "", "%.2f");
    ImPlot::SetupLegend(ImPlotLocation_NorthEast);
    ImPlot::SetupFinish();

    ImPlot::PlotLine("total", cpu_total_.data(), (int)cpu_total_.size(), 1,
                     -(double)cpu_total_.size(), 0, head_);
    ImPlot::PlotLine("prepare", cpu_prepare_.data(), (int)cpu_prepare_.size(),
                     1, -(double)cpu_prepare_.size(), 0, head_);
    ImPlot::PlotLine("solve", cpu_solve_.data(), (int)cpu_solve_.size(), 1,
                     -(double)cpu_solve_.size(), 0, head_);
    ImPlot::PlotLine("collision", cpu_collision_.data(),
                     (int)cpu_collision_.size(), 1,
                     -(double)cpu_collision_.size(), 0, head_);
    ImPlot::PlotLine("other", cpu_other_.data(), (int)cpu_other_.size(), 1,
                     -(double)cpu_other_.size(), 0, head_);
    ImPlot::PopStyleVar();
    ImPlot::EndPlot();
  }
}

void SimProfiler::DimensionsGraph(ImVec2 plot_size) {
  ImPlotFlags flags =
      ImPlot_SetupPlotFlags(plot_size) | ImPlotFlags_NoMouseText;
  if (ImPlot::BeginPlot("Dimensions vs frame", plot_size, flags)) {
    ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 2.0f);
    ImPlot_SetupTimeAxis(plot_size, "");
    ImPlot_SetupValueAxis(plot_size, "", "%.0f");
    ImPlot::SetupLegend(ImPlotLocation_NorthEast);
    ImPlot::SetupFinish();

    ImPlot::PlotLine("dof", dim_dof_.data(), (int)dim_dof_.size(), 1,
                     -(double)dim_dof_.size(), 0, head_);
    ImPlot::PlotLine("body", dim_body_.data(), (int)dim_body_.size(), 1,
                     -(double)dim_body_.size(), 0, head_);
    ImPlot::PlotLine("constraint", dim_constraint_.data(),
                     (int)dim_constraint_.size(), 1,
                     -(double)dim_constraint_.size(), 0, head_);
    ImPlot::PlotLine("sqrt(nnz)", dim_sqrt_nnz_.data(),
                     (int)dim_sqrt_nnz_.size(), 1,
                     -(double)dim_sqrt_nnz_.size(), 0, head_);
    ImPlot::PlotLine("contact", dim_contact_.data(), (int)dim_contact_.size(),
                     1, -(double)dim_contact_.size(), 0, head_);
    ImPlot::PlotLine("iteration", dim_iteration_.data(),
                     (int)dim_iteration_.size(), 1,
                     -(double)dim_iteration_.size(), 0, head_);
    ImPlot::PopStyleVar();
    ImPlot::EndPlot();
  }
}

}  // namespace mujoco::platform
