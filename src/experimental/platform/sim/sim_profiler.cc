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
#include <cmath>
#include <cstdio>
#include <string>
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

  for (std::vector<float>* v : {&cpu_total_, &cpu_collision_, &cpu_prepare_,
                                &cpu_solve_, &cpu_other_, &dim_dof_,
                                &dim_body_, &dim_constraint_, &dim_nnz_,
                                &dim_contact_, &dim_iteration_}) {
    v->assign(kMaxFrames, 0);
  }
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

  mjtNum avg_total = 1000.0 * total / number;
  cpu_total_[head_] = avg_total;

  mjtNum collision = 1000.0 * data->timer[mjTIMER_POS_COLLISION].duration / number;
  cpu_collision_[head_] = collision;

  mjtNum prepare = 1000.0 * ((data->timer[mjTIMER_POS_MAKE].duration / number) +
                             (data->timer[mjTIMER_POS_PROJECT].duration / number));
  cpu_prepare_[head_] = prepare;

  mjtNum solve = 1000.0 * data->timer[mjTIMER_CONSTRAINT].duration / number;
  cpu_solve_[head_] = solve;

  mjtNum other = avg_total - collision - prepare - solve;
  cpu_other_[head_] = other;

  // Solver diagnostics.
  mjtNum nnz = 0;
  int solver_niter = 0;
  const int nisland =
      data->nefc ? mjMAX(1, mjMIN(data->nisland, mjNISLAND)) : 0;
  for (int island = 0; island < nisland; island++) {
    nnz += data->solver_nnz[island];
    solver_niter += data->solver_niter[island];
  }

  int nv = (model->opt.enableflags & mjENBL_SLEEP) ? data->nv_awake : model->nv;
  dim_dof_[head_] = nv;

  int nbody = (model->opt.enableflags & mjENBL_SLEEP) ? data->nbody_awake
                                                      : model->nbody;
  dim_body_[head_] = nbody;

  dim_constraint_[head_] = data->nefc;
  dim_nnz_[head_] = nnz;
  dim_contact_[head_] = data->ncon;
  dim_iteration_[head_] = static_cast<float>(solver_niter) / mjMAX(1, nisland);

  head_ = (head_ + 1) % kMaxFrames;
  num_frames_ = std::min(num_frames_ + 1, kMaxFrames);
}

SimProfiler::Summary SimProfiler::GetSummary() const {
  Summary summary;
  summary.max_frames = kMaxFrames;
  summary.num_frames = num_frames_;

  // Valid data is contiguous in [0, num_frames_): the ring buffer fills from
  // index 0 and num_frames_ saturates at max_frames, so once it wraps the
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
      sum += buffer[i];
      metric.min = std::min(metric.min, buffer[i]);
      metric.max = std::max(metric.max, buffer[i]);
    }
    metric.average = static_cast<float>(sum / n);
    return metric;
  };

  // CpuTimeGraph stats.
  summary.cpu_total = stats(cpu_total_);
  summary.cpu_collision = stats(cpu_collision_);
  summary.cpu_prepare = stats(cpu_prepare_);
  summary.cpu_solve = stats(cpu_solve_);
  summary.cpu_other = stats(cpu_other_);

  // DimensionsGraph stats.
  summary.dim_dof = stats(dim_dof_);
  summary.dim_body = stats(dim_body_);
  summary.dim_constraint = stats(dim_constraint_);
  summary.dim_nnz = stats(dim_nnz_);
  summary.dim_contact = stats(dim_contact_);
  summary.dim_iteration = stats(dim_iteration_);
  return summary;
}

// Returns a dynamically padded legend label for ImPlot.
static std::string GetLegendLabel(const char* name, const std::vector<float>& values) {
  float sum = 0.f;
  int count = 0;
  for (float val : values) {
    if (val != 0.f) {
      sum += val;
      count++;
    }
  }
  float avg = count > 0 ? (sum / count) : 0.f;

  char buf[128];
  std::snprintf(buf, sizeof(buf), "%-9s (%4.0f)###%s", name, avg, name);
  return std::string(buf);
}

// Returns a dynamically padded legend label for the Dimensions plot.
static std::string GetDimensionLabel(const char* name, float value) {
  char buf[128];
  std::snprintf(buf, sizeof(buf), "%-10s: %5.0f###%s", name, value, name);
  return std::string(buf);
}

void SimProfiler::CpuTimeGraph(ImVec2 plot_size) {
  ScopedStyle style;
  style.Font(ScopedFont::kMono);

  ImPlotFlags flags =
      ImPlot_SetupPlotFlags(plot_size) | ImPlotFlags_NoMouseText;
  if (ImPlot::BeginPlot("CPU microseconds (avg) vs frame", plot_size, flags)) {
    ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 2.0f);
    ImPlot_SetupTimeAxis(plot_size, "");
    ImPlot_SetupValueAxis(plot_size, "", "%.0f");
    float max_val = 0.f;
    for (float val : cpu_total_) {
      if (val > max_val) {
        max_val = val;
      }
    }
    float rounded_max = std::max(100.0f, std::ceil(max_val / 100.0f) * 100.0f);
    ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0, rounded_max, ImPlotCond_Always);
    ImPlot::SetupLegend(ImPlotLocation_NorthEast);
    ImPlot::SetupFinish();

    ImPlot::PlotLine(GetLegendLabel("total", cpu_total_).c_str(),
                     cpu_total_.data(), (int)cpu_total_.size(), 1,
                     -(double)cpu_total_.size(), 0, head_);
    ImPlot::PlotLine(GetLegendLabel("prepare", cpu_prepare_).c_str(),
                     cpu_prepare_.data(), (int)cpu_prepare_.size(), 1,
                     -(double)cpu_prepare_.size(), 0, head_);
    ImPlot::PlotLine(GetLegendLabel("solve", cpu_solve_).c_str(),
                     cpu_solve_.data(), (int)cpu_solve_.size(), 1,
                     -(double)cpu_solve_.size(), 0, head_);
    ImPlot::PlotLine(GetLegendLabel("collision", cpu_collision_).c_str(),
                     cpu_collision_.data(), (int)cpu_collision_.size(), 1,
                     -(double)cpu_collision_.size(), 0, head_);
    ImPlot::PlotLine(GetLegendLabel("other", cpu_other_).c_str(),
                     cpu_other_.data(), (int)cpu_other_.size(), 1,
                     -(double)cpu_other_.size(), 0, head_);
    ImPlot::PopStyleVar();
    ImPlot::EndPlot();
  }
}

void SimProfiler::DimensionsGraph(ImVec2 plot_size) {
  ScopedStyle style;
  style.Font(ScopedFont::kMono);

  ImPlotFlags flags =
      ImPlot_SetupPlotFlags(plot_size) | ImPlotFlags_NoMouseText;
  if (ImPlot::BeginPlot("Dimensions: current vs frame", plot_size, flags)) {
    ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 2.0f);
    ImPlot_SetupTimeAxis(plot_size, "");
    ImPlot_SetupValueAxis(plot_size, "", "%.0f");
    ImPlot::SetupAxisScale(ImAxis_Y1, ImPlotScale_Log10);
    float max_val = 10.f;
    for (float val : dim_dof_) { max_val = std::max(max_val, val); }
    for (float val : dim_body_) { max_val = std::max(max_val, val); }
    for (float val : dim_constraint_) { max_val = std::max(max_val, val); }
    for (float val : dim_nnz_) { max_val = std::max(max_val, val); }
    for (float val : dim_contact_) { max_val = std::max(max_val, val); }
    for (float val : dim_iteration_) { max_val = std::max(max_val, val); }
    float log_max = std::log10(max_val);
    float y_max = std::pow(10.f, log_max + 0.1f);
    ImPlot::SetupAxisLimits(ImAxis_Y1, 1.0, y_max, ImPlotCond_Always);
    ImPlot::SetupLegend(ImPlotLocation_NorthEast);
    ImPlot::SetupFinish();

    // The latest sample in the ring buffer, shown in the legend labels.
    auto latest = [this](const std::vector<float>& buffer) -> float {
      return buffer[(head_ + kMaxFrames - 1) % kMaxFrames];
    };

    const int count = static_cast<int>(dim_dof_.size());
    struct GetterData {
      const std::vector<float>* vec;
      int count;
      int offset;
    };
    auto getter = +[](int idx, void* user_data) -> ImPlotPoint {
      const auto* gd = static_cast<const GetterData*>(user_data);
      float val = (*gd->vec)[(gd->offset + idx) % gd->count];
      double x = -gd->count + idx;
      double y = (val <= 0.0f) ? std::nan("") : static_cast<double>(val);
      return ImPlotPoint{x, y};
    };

    GetterData nnz_data{&dim_nnz_, count, head_};
    ImPlot::PlotLineG(GetDimensionLabel("nnz", latest(dim_nnz_)).c_str(),
                      getter, &nnz_data, count);
    GetterData constraint_data{&dim_constraint_, count, head_};
    ImPlot::PlotLineG(
        GetDimensionLabel("constraint", latest(dim_constraint_)).c_str(),
        getter, &constraint_data, count);
    GetterData dof_data{&dim_dof_, count, head_};
    ImPlot::PlotLineG(GetDimensionLabel("dof", latest(dim_dof_)).c_str(),
                      getter, &dof_data, count);
    GetterData contact_data{&dim_contact_, count, head_};
    ImPlot::PlotLineG(
        GetDimensionLabel("contact", latest(dim_contact_)).c_str(),
        getter, &contact_data, count);
    GetterData body_data{&dim_body_, count, head_};
    ImPlot::PlotLineG(GetDimensionLabel("body", latest(dim_body_)).c_str(),
                      getter, &body_data, count);
    GetterData iteration_data{&dim_iteration_, count, head_};
    ImPlot::PlotLineG(
        GetDimensionLabel("iteration", latest(dim_iteration_)).c_str(),
        getter, &iteration_data, count);
    ImPlot::PopStyleVar();
    ImPlot::EndPlot();
  }
}

}  // namespace mujoco::platform
