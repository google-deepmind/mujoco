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

#include "experimental/platform/sim_profiler.h"

#include <mujoco/mujoco.h>
#include <imgui.h>
#include <implot.h>

namespace mujoco::platform {

SimProfiler::SimProfiler() {
  Clear();
}

void SimProfiler::Clear() {
  constexpr int kProfilerMaxFrames = 200;
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
  cpu_total_.erase(cpu_total_.begin());
  cpu_total_.push_back(avg_total);

  mjtNum collision = data->timer[mjTIMER_POS_COLLISION].duration / number;
  cpu_collision_.erase(cpu_collision_.begin());
  cpu_collision_.push_back(collision);

  mjtNum prepare = (data->timer[mjTIMER_POS_MAKE].duration / number) +
                   (data->timer[mjTIMER_POS_PROJECT].duration / number);
  cpu_prepare_.erase(cpu_prepare_.begin());
  cpu_prepare_.push_back(prepare);

  mjtNum solve = data->timer[mjTIMER_CONSTRAINT].duration / number;
  cpu_solve_.erase(cpu_solve_.begin());
  cpu_solve_.push_back(solve);

  mjtNum other = avg_total - collision - prepare - solve;
  cpu_other_.erase(cpu_other_.begin());
  cpu_other_.push_back(other);

  // Solver diagnostics.
  mjtNum sqrt_nnz = 0;
  int solver_niter = 0;
  const int nisland = data->nefc ? mjMAX(1, mjMIN(data->nisland, mjNISLAND)) : 0;
  for (int island=0; island < nisland; island++) {
    sqrt_nnz += data->solver_nnz[island];
    solver_niter += data->solver_niter[island];
  }
  sqrt_nnz = mju_sqrt(sqrt_nnz);

  dim_dof_.erase(dim_dof_.begin());
  int nv = (model->opt.enableflags & mjENBL_SLEEP) ? data->nv_awake
                                                     : model->nv;
  dim_dof_.push_back(nv);

  dim_body_.erase(dim_body_.begin());
  int nbody = (model->opt.enableflags & mjENBL_SLEEP) ? data->nbody_awake
                                                        : model->nbody;
  dim_body_.push_back(nbody);

  dim_constraint_.erase(dim_constraint_.begin());
  dim_constraint_.push_back(data->nefc);

  dim_sqrt_nnz_.erase(dim_sqrt_nnz_.begin());
  dim_sqrt_nnz_.push_back(sqrt_nnz);

  dim_contact_.erase(dim_contact_.begin());
  dim_contact_.push_back(data->ncon);

  dim_iteration_.erase(dim_iteration_.begin());
  dim_iteration_.push_back(static_cast<float>(solver_niter) / nisland);
}


void SimProfiler::CpuTimeGraph() {
  if (ImPlot::BeginPlot("CPU Time", ImVec2(-1, 0), ImPlotFlags_NoMouseText)) {
    ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 2.0f);
    ImPlot::SetupAxis(ImAxis_X1, "frame", ImPlotAxisFlags_AutoFit);
    ImPlot::SetupAxis(ImAxis_Y1, "msec", ImPlotAxisFlags_AutoFit);
    ImPlot::SetupAxisFormat(ImAxis_Y1, "%.2f");
    ImPlot::SetupLegend(ImPlotLocation_NorthEast);
    ImPlot::SetupFinish();

    ImPlot::PlotLine("total", cpu_total_.data(), cpu_total_.size(), 1,
                     -(int)cpu_total_.size());
    ImPlot::PlotLine("prepare", cpu_prepare_.data(), cpu_prepare_.size(), 1,
                     -(int)cpu_prepare_.size());
    ImPlot::PlotLine("solve", cpu_solve_.data(), cpu_solve_.size(), 1,
                     -(int)cpu_solve_.size());
    ImPlot::PlotLine("collision", cpu_collision_.data(), cpu_collision_.size(),
                     1, -(int)cpu_collision_.size());
    ImPlot::PlotLine("other", cpu_other_.data(), cpu_other_.size(), 1,
                     -(int)cpu_other_.size());
    ImPlot::PopStyleVar();
    ImPlot::EndPlot();
  }
}

void SimProfiler::DimensionsGraph() {
  if (ImPlot::BeginPlot("Dimensions", ImVec2(-1, 0), ImPlotFlags_NoMouseText)) {
    ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 2.0f);
    ImPlot::SetupAxis(ImAxis_X1, "frame", ImPlotAxisFlags_AutoFit);
    ImPlot::SetupAxis(ImAxis_Y1, "count", ImPlotAxisFlags_AutoFit);
    ImPlot::SetupAxisFormat(ImAxis_Y1, "%.0f");
    ImPlot::SetupLegend(ImPlotLocation_NorthEast);
    ImPlot::SetupFinish();

    ImPlot::PlotLine("dof", dim_dof_.data(), dim_dof_.size(), 1,
                     -(int)dim_dof_.size());
    ImPlot::PlotLine("body", dim_body_.data(), dim_body_.size(), 1,
                     -(int)dim_body_.size());
    ImPlot::PlotLine("constraint", dim_constraint_.data(),
                     dim_constraint_.size(), 1, -(int)dim_constraint_.size());
    ImPlot::PlotLine("sqrt(nnz)", dim_sqrt_nnz_.data(), dim_sqrt_nnz_.size(), 1,
                     -(int)dim_sqrt_nnz_.size());
    ImPlot::PlotLine("contact", dim_contact_.data(), dim_contact_.size(), 1,
                     -(int)dim_contact_.size());
    ImPlot::PlotLine("iteration", dim_iteration_.data(), dim_iteration_.size(),
                     1, -(int)dim_iteration_.size());
    ImPlot::PopStyleVar();
    ImPlot::EndPlot();
  }
}

}  // namespace mujoco::platform
