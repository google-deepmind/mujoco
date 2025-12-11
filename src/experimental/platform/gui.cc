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

#include "experimental/platform/gui.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include <imgui.h>
#include <imgui_internal.h>
#include <implot.h>
#include <mujoco/mujoco.h>
#include "experimental/platform/helpers.h"
#include "experimental/platform/imgui_widgets.h"

namespace mujoco::platform {

static constexpr int kToolsBarHeight = 48;
static constexpr int kStatusBarHeight = 32;
static constexpr float kOptionsRelWidth = 0.22f;
static constexpr float kInspectorRelWidth = 0.22f;
static constexpr float kStatsRelHeight = 0.3f;

static ImVec2 GetFlexElementSize(int num_cols) {
  const float width = (ImGui::GetContentRegionAvail().x / num_cols) -
                      ImGui::GetStyle().FramePadding.x * 2;
  return ImVec2(width, 0);
}

void SetupTheme(GuiTheme theme) {
  ImGuiStyle& s = ImGui::GetStyle();
  ImVec4* c = s.Colors;
  if (theme == GuiTheme::kDark) {
    ImGui::StyleColorsDark(&s);
    c[ImGuiCol_Text]                   = ImVec4(1.00, 1.00, 1.00, 1.00);
    c[ImGuiCol_TextDisabled]           = ImVec4(0.40, 0.40, 0.40, 1.00);
    c[ImGuiCol_ChildBg]                = ImVec4(0.25, 0.25, 0.25, 1.00);
    c[ImGuiCol_WindowBg]               = ImVec4(0.25, 0.25, 0.25, 1.00);
    c[ImGuiCol_PopupBg]                = ImVec4(0.25, 0.25, 0.25, 1.00);
    c[ImGuiCol_Border]                 = ImVec4(0.12, 0.12, 0.12, 0.71);
    c[ImGuiCol_BorderShadow]           = ImVec4(1.00, 1.00, 1.00, 0.06);
    c[ImGuiCol_FrameBg]                = ImVec4(0.42, 0.42, 0.42, 0.54);
    c[ImGuiCol_FrameBgHovered]         = ImVec4(0.42, 0.42, 0.42, 0.40);
    c[ImGuiCol_FrameBgActive]          = ImVec4(0.56, 0.56, 0.56, 0.67);
    c[ImGuiCol_TitleBg]                = ImVec4(0.19, 0.19, 0.19, 1.00);
    c[ImGuiCol_TitleBgActive]          = ImVec4(0.22, 0.22, 0.22, 1.00);
    c[ImGuiCol_TitleBgCollapsed]       = ImVec4(0.17, 0.17, 0.17, 0.90);
    c[ImGuiCol_MenuBarBg]              = ImVec4(0.34, 0.34, 0.34, 1.00);
    c[ImGuiCol_ScrollbarBg]            = ImVec4(0.24, 0.24, 0.24, 0.53);
    c[ImGuiCol_ScrollbarGrab]          = ImVec4(0.41, 0.41, 0.41, 1.00);
    c[ImGuiCol_ScrollbarGrabHovered]   = ImVec4(0.52, 0.52, 0.52, 1.00);
    c[ImGuiCol_ScrollbarGrabActive]    = ImVec4(0.76, 0.76, 0.76, 1.00);
    c[ImGuiCol_CheckMark]              = ImVec4(0.65, 0.65, 0.65, 1.00);
    c[ImGuiCol_SliderGrab]             = ImVec4(0.52, 0.52, 0.52, 1.00);
    c[ImGuiCol_SliderGrabActive]       = ImVec4(0.64, 0.64, 0.64, 1.00);
    c[ImGuiCol_Button]                 = ImVec4(0.54, 0.54, 0.54, 0.35);
    c[ImGuiCol_ButtonHovered]          = ImVec4(0.52, 0.52, 0.52, 0.59);
    c[ImGuiCol_ButtonActive]           = ImVec4(0.76, 0.76, 0.76, 1.00);
    c[ImGuiCol_Header]                 = ImVec4(0.38, 0.38, 0.38, 1.00);
    c[ImGuiCol_HeaderHovered]          = ImVec4(0.47, 0.47, 0.47, 1.00);
    c[ImGuiCol_HeaderActive]           = ImVec4(0.76, 0.76, 0.76, 0.77);
    c[ImGuiCol_Separator]              = ImVec4(0.00, 0.00, 0.00, 0.18);
    c[ImGuiCol_SeparatorHovered]       = ImVec4(0.70, 0.67, 0.60, 0.29);
    c[ImGuiCol_SeparatorActive]        = ImVec4(0.70, 0.67, 0.60, 0.67);
    c[ImGuiCol_ResizeGrip]             = ImVec4(0.26, 0.59, 0.98, 0.25);
    c[ImGuiCol_ResizeGripHovered]      = ImVec4(0.26, 0.59, 0.98, 0.67);
    c[ImGuiCol_ResizeGripActive]       = ImVec4(0.26, 0.59, 0.98, 0.95);
    c[ImGuiCol_PlotLines]              = ImVec4(0.61, 0.61, 0.61, 1.00);
    c[ImGuiCol_PlotLinesHovered]       = ImVec4(1.00, 0.43, 0.35, 1.00);
    c[ImGuiCol_PlotHistogram]          = ImVec4(0.90, 0.70, 0.00, 1.00);
    c[ImGuiCol_PlotHistogramHovered]   = ImVec4(1.00, 0.60, 0.00, 1.00);
    c[ImGuiCol_TextSelectedBg]         = ImVec4(0.73, 0.73, 0.73, 0.35);
    c[ImGuiCol_ModalWindowDimBg]       = ImVec4(0.80, 0.80, 0.80, 0.35);
    c[ImGuiCol_DragDropTarget]         = ImVec4(1.00, 1.00, 0.00, 0.90);
    c[ImGuiCol_NavHighlight]           = ImVec4(0.26, 0.59, 0.98, 1.00);
    c[ImGuiCol_NavWindowingHighlight]  = ImVec4(1.00, 1.00, 1.00, 0.70);
    c[ImGuiCol_NavWindowingDimBg]      = ImVec4(0.80, 0.80, 0.80, 0.20);
    c[ImGuiCol_DockingEmptyBg]         = ImVec4(0.38, 0.38, 0.38, 1.00);
    c[ImGuiCol_Tab]                    = ImVec4(0.25, 0.25, 0.25, 1.00);
    c[ImGuiCol_TabHovered]             = ImVec4(0.40, 0.40, 0.40, 1.00);
    c[ImGuiCol_TabActive]              = ImVec4(0.33, 0.33, 0.33, 1.00);
    c[ImGuiCol_TabUnfocused]           = ImVec4(0.25, 0.25, 0.25, 1.00);
    c[ImGuiCol_TabUnfocusedActive]     = ImVec4(0.33, 0.33, 0.33, 1.00);
    c[ImGuiCol_DockingPreview]         = ImVec4(0.85, 0.85, 0.85, 0.28);
    c[ImGuiCol_WindowBg].w = 1.0f;
  } else if (theme == GuiTheme::kLight) {
    ImGui::StyleColorsLight(&s);
    ImVec4 white       = ImVec4(1.00, 1.00, 1.00, 1.00);
    ImVec4 transparent = ImVec4(0.00, 0.00, 0.00, 0.00);
    ImVec4 dark        = ImVec4(0.00, 0.00, 0.00, 0.20);
    ImVec4 darker      = ImVec4(0.00, 0.00, 0.00, 0.50);
    ImVec4 background  = ImVec4(0.95, 0.95, 0.95, 1.00);
    ImVec4 text        = ImVec4(0.10, 0.10, 0.10, 1.00);
    ImVec4 border      = ImVec4(0.60, 0.60, 0.60, 1.00);
    ImVec4 grab        = ImVec4(0.69, 0.69, 0.69, 1.00);
    ImVec4 header      = ImVec4(0.86, 0.86, 0.86, 1.00);
    ImVec4 active      = ImVec4(0.00, 0.47, 0.84, 1.00);
    ImVec4 hover       = ImVec4(0.00, 0.47, 0.84, 0.20);

    c[ImGuiCol_Text] = text;
    c[ImGuiCol_WindowBg] = background;
    c[ImGuiCol_ChildBg] = background;
    c[ImGuiCol_PopupBg] = white;
    c[ImGuiCol_Border] = border;
    c[ImGuiCol_BorderShadow] = transparent;
    c[ImGuiCol_Button] = header;
    c[ImGuiCol_ButtonHovered] = hover;
    c[ImGuiCol_ButtonActive] = active;
    c[ImGuiCol_FrameBg] = white;
    c[ImGuiCol_FrameBgHovered] = hover;
    c[ImGuiCol_FrameBgActive] = active;
    c[ImGuiCol_MenuBarBg] = header;
    c[ImGuiCol_Header] = header;
    c[ImGuiCol_HeaderHovered] = hover;
    c[ImGuiCol_HeaderActive] = active;
    c[ImGuiCol_CheckMark] = text;
    c[ImGuiCol_SliderGrab] = grab;
    c[ImGuiCol_SliderGrabActive] = darker;
    c[ImGuiCol_ScrollbarBg] = header;
    c[ImGuiCol_ScrollbarGrab] = grab;
    c[ImGuiCol_ScrollbarGrabHovered] = dark;
    c[ImGuiCol_ScrollbarGrabActive] = darker;
  } else  {
    ImGui::StyleColorsDark(&s);
    ImVec4 black         = ImVec4(0.00, 0.00, 0.00, 1.0);
    ImVec4 window        = ImVec4(0.25, 0.25, 0.25, 1.0);
    ImVec4 font_active   = ImVec4(1.00, 1.00, 1.00, 1.0);
    ImVec4 font_inactive = ImVec4(0.50, 0.50, 0.50, 1.0);
    ImVec4 thumb         = ImVec4(0.12, 0.12, 0.12, 1.0);
    ImVec4 section       = ImVec4(0.40, 0.15, 0.15, 1.0);
    ImVec4 button        = ImVec4(0.60, 0.40, 0.40, 1.0);
    ImVec4 check         = ImVec4(0.40, 0.40, 0.70, 1.0);
    ImVec4 frame         = ImVec4(0.40, 0.30, 0.40, 1.0);
    ImVec4 slider        = ImVec4(0.60, 0.40, 0.60, 1.0);

    c[ImGuiCol_WindowBg]               = window;
    c[ImGuiCol_ChildBg]                = black;
    c[ImGuiCol_PopupBg]                = window;
    c[ImGuiCol_Text]                   = font_active;
    c[ImGuiCol_TextDisabled]           = font_inactive;
    c[ImGuiCol_CheckMark]              = font_active;
    c[ImGuiCol_Header]                 = section;
    c[ImGuiCol_HeaderHovered]          = section;
    c[ImGuiCol_HeaderActive]           = section;
    c[ImGuiCol_TitleBgActive]          = window;
    c[ImGuiCol_ScrollbarBg]            = window;
    c[ImGuiCol_ScrollbarGrab]          = thumb;
    c[ImGuiCol_ScrollbarGrabHovered]   = thumb;
    c[ImGuiCol_ScrollbarGrabActive]    = thumb;
    c[ImGuiCol_FrameBg]                = frame;
    c[ImGuiCol_FrameBgHovered]         = frame;
    c[ImGuiCol_FrameBgActive]          = frame;
    c[ImGuiCol_SliderGrab]             = slider;
    c[ImGuiCol_SliderGrabActive]       = slider;
    c[ImGuiCol_Button]                 = window;
    c[ImGuiCol_ButtonHovered]          = button;
    c[ImGuiCol_ButtonActive]           = button;
    c[ImGuiCol_Tab]                    = window;
    c[ImGuiCol_TabHovered]             = check;
    c[ImGuiCol_TabSelected]            = check;
    c[ImGuiCol_TabDimmed]              = window;
    c[ImGuiCol_TabDimmedSelected]      = check;
  }

  int hspacing = 4;
  int vspacing = 6;
  float rounding = 4.0f;
  s.DisplaySafeAreaPadding = ImVec2(0, 0);
  s.WindowPadding = ImVec2(hspacing, vspacing);
  s.FramePadding = ImVec2(hspacing, vspacing);
  s.ItemSpacing = ImVec2(hspacing, vspacing);
  s.ItemInnerSpacing = ImVec2(hspacing, vspacing);
  s.WindowRounding = rounding;
  s.FrameRounding = rounding;
  s.TabRounding = rounding;
  s.ScrollbarRounding = rounding;
  s.ChildRounding = rounding;
  s.GrabRounding = rounding;
  s.PopupRounding = rounding;
  s.WindowBorderSize = 0.0f;
  s.FrameBorderSize = 1.0f;
  s.PopupBorderSize = 1.0f;
  s.IndentSpacing = 20.0f;
  s.ScrollbarSize = 12.0f;
  s.GrabMinSize = 5.0f;
  s.WindowMenuButtonPosition = ImGuiDir_None;
  s.TabCloseButtonMinWidthSelected = 0.0f;
  s.DockingNodeHasCloseButton = false;
}

ImVec4 ConfigureDockingLayout() {
  ImGuiViewport* viewport = ImGui::GetMainViewport();

  const ImVec2 dockspace_pos{
      viewport->WorkPos.x,
      viewport->WorkPos.y + kToolsBarHeight
  };
  const ImVec2 dockspace_size{
      viewport->WorkSize.x,
      viewport->WorkSize.y - kToolsBarHeight - kStatusBarHeight
  };

  ImGuiID root = ImGui::GetID("Root");
  const bool first_time = (ImGui::DockBuilderGetNode(root) == nullptr);

  if (first_time) {
    ImGui::DockBuilderRemoveNode(root);
    ImGui::DockBuilderAddNode(root, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(root, dockspace_size);

    // Slice up the main dock space.
    ImGuiID main = root;

    ImGuiID options = 0;
    ImGui::DockBuilderSplitNode(main, ImGuiDir_Left, kOptionsRelWidth,
                                &options, &main);

    ImGuiID inspector = 0;
    ImGui::DockBuilderSplitNode(main, ImGuiDir_Right, kInspectorRelWidth,
                                &inspector, &main);

    ImGuiID stats = 0;
    ImGui::DockBuilderSplitNode(options, ImGuiDir_Down, kStatsRelHeight,
                                &stats, &options);

    ImGuiID properties = 0;
    ImGui::DockBuilderSplitNode(inspector, ImGuiDir_Down, kStatsRelHeight,
                                &properties, &inspector);

    ImGui::DockBuilderDockWindow("Dockspace", main);
    ImGui::DockBuilderDockWindow("Options", options);
    ImGui::DockBuilderDockWindow("Explorer", inspector);
    ImGui::DockBuilderDockWindow("Inspector", inspector);
    ImGui::DockBuilderDockWindow("Properties", properties);
    ImGui::DockBuilderDockWindow("Stats", stats);
    ImGui::DockBuilderFinish(root);
  }

  // Create a dummy window filling the entire workspace in which we can perform
  // docking.
  ImGui::SetNextWindowPos(dockspace_pos);
  ImGui::SetNextWindowSize(dockspace_size);
  ImGui::SetNextWindowViewport(viewport->ID);

  const ImGuiWindowFlags kWorkspaceFlags =
      ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
      ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
      ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoBringToFrontOnFocus |
      ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_NoBackground;

  const ImGuiWindowFlags kFixedFlags = ImGuiWindowFlags_NoTitleBar |
                                       ImGuiWindowFlags_NoMove |
                                       ImGuiWindowFlags_NoResize |
                                       ImGuiWindowFlags_NoScrollbar |
                                       ImGuiWindowFlags_NoDocking;

  // Main workspace area in which we can dock other windows.
  {
    platform::ScopedStyle style;
    style.Var(ImGuiStyleVar_WindowRounding, 0.0f);
    style.Var(ImGuiStyleVar_WindowBorderSize, 0.0f);
    style.Var(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::Begin("Dockspace", nullptr, kWorkspaceFlags);
    const ImGuiDockNodeFlags kDockSpaceFlags =
        ImGuiDockNodeFlags_PassthruCentralNode |
        ImGuiDockNodeFlags_NoDockingOverCentralNode |
        ImGuiDockNodeFlags_AutoHideTabBar;
    ImGui::DockSpace(root, ImVec2(0.0f, 0.0f), kDockSpaceFlags);
    ImGui::End();
  }

  // Toolbar is fixed at the top.
  {
    platform::ScopedStyle style;
    style.Var(ImGuiStyleVar_WindowBorderSize, 1.0f);
    style.Var(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::SetNextWindowPos(viewport->WorkPos, ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(viewport->Size.x, kToolsBarHeight), ImGuiCond_Always);
    ImGui::Begin("ToolBar", nullptr, kFixedFlags);
    ImGui::End();
  }

  // StatusBar is fixed at the bottom.
  {
    platform::ScopedStyle style;
    style.Var(ImGuiStyleVar_WindowBorderSize, 1.0f);
    style.Var(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::SetNextWindowPos(ImVec2(0, viewport->Size.y - kStatusBarHeight), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(viewport->Size.x, kStatusBarHeight), ImGuiCond_Always);
    ImGui::Begin("StatusBar", nullptr, kFixedFlags);
    ImGui::End();
  }

  const int settings_width = dockspace_size.x * kOptionsRelWidth;
  const int inspector_width = dockspace_size.x * kInspectorRelWidth;
  const float workspace_x = dockspace_pos.x + settings_width;
  const float workspace_y = dockspace_pos.y;
  const float workspace_w = dockspace_size.x - settings_width - inspector_width;
  const float workspace_h = dockspace_size.y;
  return ImVec4(workspace_x, workspace_y, workspace_w, workspace_h);
}

void SensorGui(const mjModel* model, const mjData* data) {
  if (model->nsensor == 0) {
    return;
  }

  ImPlot::PushStyleVar(ImPlotStyleVar_FitPadding, ImVec2(0.1f, 0.1f));
  if (ImPlot::BeginPlot("Sensors", ImVec2(-1, 0),
                        ImPlotFlags_NoLegend | ImPlotFlags_NoMouseText)) {
    ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_None);

    ImPlot::SetupAxis(ImAxis_X1, "sensor",
                      ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_NoLabel);
    ImPlot::SetupAxisLimits(ImAxis_X1, 0, 5, ImPlotCond_Once);

    ImPlot::SetupAxis(ImAxis_Y1, "value",
                      ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_NoLabel);
    ImPlot::SetupAxisFormat(ImAxis_Y1, "%.1f");
    ImPlot::SetupAxisLimits(ImAxis_Y1, -100, 100, ImPlotCond_Once);
    ImPlot::SetupFinish();

    // The values to be plotted.
    std::vector<ImPlotPoint> sensor_values;

    // The x-value of the bar to be plotted. Multiple bars will belong to the
    // same sensor (i.e. the sensor_dim), but each group of bars will be appear
    // in sequence along the x-axis.
    float x_value = 0.f;

    // The index of the sensor being plotted, based on sensor_type.
    int sensor_index = 0;

    // Function that plots the current group of sensor bars.
    auto plot_lines = [](int sensor_idx, const ImPlotPoint* values, int count) {
      constexpr float bar_weight = 5.0f;
      ImPlot::SetNextLineStyle(IMPLOT_AUTO_COL, bar_weight);
      std::string sensor_label = "Sensor " + std::to_string(sensor_idx);
      ImPlot::PlotLine(sensor_label.c_str(), &values->x, &values->y, count,
                       ImPlotLineFlags_Segments, 0, 2 * sizeof(double));
    };

    for (int n = 0; n < model->nsensor; n++) {
      if (n > 0 && model->sensor_type[n] != model->sensor_type[n - 1]) {
        plot_lines(sensor_index, sensor_values.data(), sensor_values.size());
        sensor_values.clear();
        ++sensor_index;
      }

      const int adr = model->sensor_adr[n];
      const int dim = model->sensor_dim[n];
      const mjtNum cutoff =
          (model->sensor_cutoff[n] > 0 ? model->sensor_cutoff[n] : 1);
      for (int i = 0; i < dim; ++i) {
        sensor_values.push_back({x_value, 0});
        sensor_values.push_back({x_value, data->sensordata[adr + i] / cutoff});
        x_value += 1.f;
      }
    }

    // Ensure the last group of sensors is plotted.
    plot_lines(sensor_index, sensor_values.data(), sensor_values.size());
    ImPlot::EndPlot();
  }
  ImPlot::PopStyleVar();
}

void StateGui(const mjModel* model, mjData* data, std::vector<mjtNum>& state,
              int& state_sig, float min_width) {
  const float available_width =
      ImGui::GetContentRegionAvail().x - ImGui::GetTreeNodeToLabelSpacing();
  const int num_cols = std::clamp(
      static_cast<int>(std::floor(available_width / min_width)), 1, 4);
  const ImVec2 size = GetFlexElementSize(num_cols);

  ImGui::Unindent(0.5f * ImGui::GetTreeNodeToLabelSpacing());
  // State component names and tooltips.
  static constexpr const char* name_and_tooltip[][2] = {
      {"TIME", "Time"},
      {"QPOS", "Position"},
      {"QVEL", "Velocity"},
      {"ACT", "Actuator activation"},
      {"WARMSTART", "Acceleration used for warmstart"},
      {"CTRL", "Control"},
      {"QFRC_APPLIED", "Applied generalized force"},
      {"XFRC_APPLIED", "Applied Cartesian force/torque"},
      {"EQ_ACTIVE", "Enable/disable constraints"},
      {"MOCAP_POS", "Positions of mocap bodies"},
      {"MOCAP_QUAT", "Orientations of mocap bodies"},
      {"USERDATA", "User data"},
      {"PLUGIN", "Plugin state"},
  };

  int prev_state_sig = state_sig;

  // State component checkboxes.
  if (ImGui::BeginTable("##StateSignature", num_cols)) {
    for (int i = 0; i < mjNSTATE; ++i) {
      ImGui::TableNextColumn();
      bool checked = state_sig & (1 << i);
      ImGui::Checkbox(name_and_tooltip[i][0], &checked);
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("%s", name_and_tooltip[i][1]);
      }
      state_sig = checked ? (state_sig | (1 << i)) : (state_sig & ~(1 << i));
    }
    ImGui::EndTable();
  }

  // Buttons to select commonly used state signatures.
  if (ImGui::BeginTable("##CommonSignatures", num_cols)) {
    ImGui::TableNextColumn();
    if (ImGui::Button("Physics", size)) {
      state_sig = (state_sig == mjSTATE_PHYSICS) ? 0 : mjSTATE_PHYSICS;
    }
    ImGui::TableNextColumn();
    if (ImGui::Button("Full Physics", size)) {
      state_sig = (state_sig == mjSTATE_FULLPHYSICS) ? 0 : mjSTATE_FULLPHYSICS;
    }
    ImGui::TableNextColumn();
    if (ImGui::Button("User", size)) {
      state_sig = (state_sig == mjSTATE_USER) ? 0 : mjSTATE_USER;
    }
    ImGui::TableNextColumn();
    if (ImGui::Button("Integration", size)) {
      state_sig = (state_sig == mjSTATE_INTEGRATION) ? 0 : mjSTATE_INTEGRATION;
    }
    ImGui::EndTable();
  }

  if (state_sig != prev_state_sig) {
    const int size = mj_stateSize(model, state_sig);
    state.resize(size);
  }

  if (state.empty()) {
    // The state size is 0, let the user know why.
    ImGui::Separator();
    ImGui::BeginDisabled();
    ImGui::TextWrapped(
        state_sig == 0
            ? "No state components are selected."
            : "Selected state components do not exist in the model.");
    ImGui::EndDisabled();
  } else {
    mj_getState(model, data, state.data(), state_sig);
    bool changed = false;

    if (ImGui::BeginTable(
            "State", 3,
            ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersOuter |
                ImGuiTableFlags_BordersV | ImGuiTableFlags_Resizable |
                ImGuiTableFlags_ScrollY,
            ImVec2(0, ImGui::GetTextLineHeightWithSpacing() * 20))) {
      ImGui::TableSetupColumn("Index");
      ImGui::TableSetupColumn("Name");
      ImGui::TableSetupColumn("Value");
      ImGui::TableSetupScrollFreeze(0, 1);
      ImGui::TableHeadersRow();

      ImGuiListClipper clipper;
      clipper.Begin(state.size());
      while (clipper.Step()) {
        int global = 0;
        for (int i = 0; i < mjNSTATE; ++i) {
          if (state_sig & (1 << i)) {
            for (int local = 0; local < mj_stateSize(model, (1 << i));
                 ++local, ++global) {
              if (global < clipper.DisplayStart) {
                continue;
              }
              if (global >= clipper.DisplayEnd) {
                break;
              }
              ImGui::TableNextRow();

              ImGui::TableNextColumn();
              ImGui::Text("%d", global);

              ImGui::TableNextColumn();
              ImGui::Text("%s[%d]", name_and_tooltip[i][0], local);

              ImGui::TableNextColumn();
              float value = state[global];
              ImGui::PushItemWidth(-std::numeric_limits<float>::min());
              ImGui::PushID(global);
              if (ImGui::DragFloat("##value", &value, 0.01f, 0, 0, "%.3f")) {
                changed = true;
              }
              ImGui::PopID();
              ImGui::PopItemWidth();
              state[global] = value;
            }
          }
        }
      }
      ImGui::EndTable();
    }

    if (changed) {
      mj_setState(model, data, state.data(), state_sig);
    }
  }

  ImGui::Indent(0.5f * ImGui::GetTreeNodeToLabelSpacing());
}

void WatchGui(const mjModel* model, const mjData* data, char* field_name,
              int field_len, int& field_index) {
  const float item_width = ImGui::GetWindowWidth() * .6f;
  ImGui::PushItemWidth(item_width);

  ImGui::InputText("Field", field_name, field_len);
  ImGui::InputInt("Index", &field_index);
  const mjtNum* value = static_cast<const mjtNum*>(
      GetValue(model, data, field_name, field_index));

  ScopedStyle style;
  style.Color(ImGuiCol_FrameBg, ImGui::GetStyle().Colors[ImGuiCol_WindowBg]);

  if (value) {
    char buf[100];
    int size = std::snprintf(buf, sizeof(buf), "%0.3f", *value);
    ImGui::InputText("Value", buf, size, ImGuiInputTextFlags_ReadOnly);
  } else {
    ImGui::BeginDisabled();
    style.Color(ImGuiCol_Text, ImColor(255, 0, 0, 255));
    char buf[] = "Invalid field/index!";
    ImGui::InputText("Value", buf, sizeof(buf), ImGuiInputTextFlags_ReadOnly);
    ImGui::EndDisabled();
  }

  ImGui::PopItemWidth();
}

void PhysicsGui(mjModel* model, float min_width) {
  const float available_width =
      ImGui::GetContentRegionAvail().x - ImGui::GetTreeNodeToLabelSpacing();
  const int num_cols = std::clamp(
      static_cast<int>(std::floor(available_width / min_width)), 1, 6);

  const float item_width = ImGui::GetWindowWidth() * .6f;
  ImGui::PushItemWidth(item_width);

  auto& opt = model->opt;

  const char* opts0[] = {"Euler", "RK4", "implicit", "implicitfast"};
  ImGui::Combo("Integrator", &opt.integrator, opts0, IM_ARRAYSIZE(opts0));

  const char* opts1[] = {"Pyramidal", "Elliptic"};
  ImGui::Combo("Cone", &opt.cone, opts1, IM_ARRAYSIZE(opts1));

  const char* opts2[] = {"Dense", "Sparse", "Auto"};
  ImGui::Combo("Jacobian", &opt.jacobian, opts2, IM_ARRAYSIZE(opts2));

  const char* opts3[] = {"PGS", "CG", "Newton"};
  ImGui::Combo("Solver", &opt.solver, opts3, IM_ARRAYSIZE(opts3));

  if (ImGui::TreeNodeEx("Flags", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::BeginTable("##PhysicsFlagsTable", num_cols)) {
      const ImVec2 size = GetFlexElementSize(num_cols);
      for (int i = 0; i < mjNDISABLE; ++i) {
        ImGui::TableNextColumn();
        int flipped = ~opt.disableflags;
        ImGui_BitToggle(mjDISABLESTRING[i], &flipped, 1 << i, size);
        opt.disableflags = ~flipped;
      }
      for (int i = 0; i < mjNENABLE; ++i) {
        ImGui::TableNextColumn();
        ImGui_BitToggle(mjENABLESTRING[i], &opt.enableflags, 1 << i, size);
      }
      ImGui::EndTable();
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Actuator Groups")) {
    if (ImGui::BeginTable("##ActuatorGroupsTable", num_cols)) {
      const ImVec2 size = GetFlexElementSize(num_cols);
      for (int i = 0; i < 6; ++i) {
        char label[64];
        std::snprintf(label, sizeof(label), "Act Group %d", i);
        ImGui::TableNextColumn();
        int flipped = ~opt.disableactuator;
        ImGui_BitToggle(label, &flipped, 1 << i, size);
        opt.disableactuator = ~flipped;
      }
      ImGui::EndTable();
    }
    ImGui::TreePop();
  };

  if (ImGui::TreeNodeEx("Algorithmic Parameters")) {
    ImGui_Input("Timestep", &opt.timestep, {0, 1, 0.01, 0.1});
    ImGui_Input("Iterations", &opt.iterations, {0, 1000, 1, 10});
    ImGui_Input("Tolerance", &opt.tolerance, {0, 1, 1e-7, 1e-6});
    ImGui_Input("LS Iter", &opt.ls_iterations, {0, 100, 1, 0.1});
    ImGui_Input("LS Tol", &opt.ls_tolerance, {0, 0.1, 0.01, 0.1});
    ImGui_Input("Noslip Iter", &opt.noslip_iterations, {0, 1000, 1, 100});
    ImGui_Input("Noslip Tol", &opt.noslip_tolerance, {0, 1, 0.01, 0.1});
    ImGui_Input("CCD Iter", &opt.ccd_iterations, {0, 1000, 1, 100});
    ImGui_Input("CCD Tol", &opt.ccd_tolerance, {0, 1, 0.01, 0.1});
    ImGui_Input("Sleep Tol", &opt.sleep_tolerance, {0, 1, 0.01, 0.1});
    ImGui_Input("SDF Iter", &opt.sdf_iterations, {1, 20, 1, 10});
    ImGui_Input("SDF Init", &opt.sdf_initpoints, {1, 100, 1, 10});
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Physical Parameters")) {
    ImGui_InputN("Gravity", opt.gravity, 3);
    ImGui_InputN("Wind", opt.wind, 3);
    ImGui_InputN("Magnetic", opt.magnetic, 3);
    ImGui_Input("Density", &opt.density, {.min = .1, .max = 1});
    ImGui_Input("Viscosity", &opt.viscosity, {.min = .1, .max = 1});
    ImGui_Input("Imp Ratio", &opt.impratio, {.min = .1, .max = 1});
    ImGui::TreePop();
  };

  if (ImGui::TreeNodeEx("Contact Override")) {
    ImGui_Input("Margin", &opt.o_margin, {.min = 0.1, .max = 1});
    ImGui_InputN("Sol Imp", opt.o_solimp, 5, {.format = "%0.1f"});
    ImGui_InputN("Sol Ref", opt.o_solref, 2, {.format = "%0.1f"});
    ImGui_InputN("Friction", opt.o_friction, 5, {.format = "%.1f"});
    ImGui::TreePop();
  }

  ImGui::PopItemWidth();
}

void VisualizationGui(mjModel* model, mjvOption* vis_options, mjvCamera* camera,
                      float min_width) {
  auto& vis = model->vis;
  auto& stat = model->stat;

  const float item_width = ImGui::GetWindowWidth() * .6f;
  ImGui::PushItemWidth(item_width);

  ImGui::SliderInt("Tree depth", &vis_options->bvh_depth, 0, 20);
  ImGui::SliderInt("Flex layer", &vis_options->flex_layer, 0, 10);

  if (ImGui::TreeNodeEx("Headlight")) {
    ImGui_SwitchToggle("Active", &vis.headlight.active);
    ImGui::ColorEdit3("Ambient", vis.headlight.ambient);
    ImGui::ColorEdit3("Diffuse", vis.headlight.diffuse);
    ImGui::ColorEdit3("Specular", vis.headlight.specular);
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Free Camera")) {
    ImGui_SwitchToggle("Orthographic", &vis.global.orthographic);
    ImGui_Input("FOV", &vis.global.fovy, {.format = "%0.2f"});
    ImGui_InputN("Center", stat.center, 3, {.format = "%0.2f"});
    ImGui_Input("Azimuth", &vis.global.azimuth, {.format = "%0.2f"});
    ImGui_Input("Elevation", &vis.global.elevation, {.format = "%0.2f"});
    if (ImGui::Button("Align")) {
      mjv_defaultFreeCamera(model, camera);
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Global")) {
    ImGui_Input("Extent", &stat.extent);
    const char* opts[] = {"Box", "Ellipsoid"};
    ImGui::SliderInt("Inertia", &vis.global.ellipsoidinertia, 0, 1,
                     opts[vis.global.ellipsoidinertia]);
    ImGui_ButtonToggle("BVH active", &vis.global.bvactive);
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Mapping")) {
    ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.3f);
    ImGui_Input("Stiffness", &vis.map.stiffness);
    ImGui_Input("Rot stiffness", &vis.map.stiffnessrot);
    ImGui_Input("Force", &vis.map.force);
    ImGui_Input("Torque", &vis.map.torque);
    ImGui_Input("Alpha", &vis.map.alpha);
    ImGui_Input("Fog start", &vis.map.fogstart);
    ImGui_Input("Fog end", &vis.map.fogend);
    ImGui_Input("Z near", &vis.map.znear);
    ImGui_Input("Z far", &vis.map.zfar);
    ImGui_Input("Haze", &vis.map.haze);
    ImGui_Input("Shadow clip", &vis.map.shadowclip);
    ImGui_Input("Shadow scale", &vis.map.shadowscale);
    ImGui::PopItemWidth();
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Scale")) {
    ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.3f);
    ImGui_Input("All (meansize)", &stat.meansize, {.format = "%0.3f"});
    ImGui_Input("Force width", &vis.scale.forcewidth);
    ImGui_Input("Contact width", &vis.scale.contactwidth);
    ImGui_Input("Contact height", &vis.scale.contactheight);
    ImGui_Input("Connect", &vis.scale.connect);
    ImGui_Input("Com", &vis.scale.com);
    ImGui_Input("Camera", &vis.scale.camera);
    ImGui_Input("Light", &vis.scale.light);
    ImGui_Input("Select point", &vis.scale.selectpoint);
    ImGui_Input("Joint length", &vis.scale.jointlength);
    ImGui_Input("Joint width", &vis.scale.jointwidth);
    ImGui_Input("Actuator length", &vis.scale.actuatorlength);
    ImGui_Input("Actuator width", &vis.scale.actuatorwidth);
    ImGui_Input("Frame length", &vis.scale.framelength);
    ImGui_Input("Frame width", &vis.scale.framewidth);
    ImGui_Input("Constraint", &vis.scale.constraint);
    ImGui_Input("Slider-crank", &vis.scale.slidercrank);
    ImGui::PopItemWidth();
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Colors")) {
    ImGui::ColorEdit4("Fog", vis.rgba.fog);
    ImGui::ColorEdit4("Haze", vis.rgba.haze);
    ImGui::ColorEdit4("Force", vis.rgba.force);
    ImGui::ColorEdit4("Inertia", vis.rgba.inertia);
    ImGui::ColorEdit4("Joint", vis.rgba.joint);
    ImGui::ColorEdit4("Actuator", vis.rgba.actuator);
    ImGui::ColorEdit4("Act. Negative", vis.rgba.actuatornegative);
    ImGui::ColorEdit4("Act. Positive", vis.rgba.actuatorpositive);
    ImGui::ColorEdit4("Center of Mass", vis.rgba.com);
    ImGui::ColorEdit4("Camera", vis.rgba.camera);
    ImGui::ColorEdit4("Light", vis.rgba.light);
    ImGui::ColorEdit4("Select Point", vis.rgba.selectpoint);
    ImGui::ColorEdit4("Auto Connect", vis.rgba.connect);
    ImGui::ColorEdit4("Contact Point", vis.rgba.contactpoint);
    ImGui::ColorEdit4("Contact Force", vis.rgba.contactforce);
    ImGui::ColorEdit4("Contact Friction", vis.rgba.contactfriction);
    ImGui::ColorEdit4("Contact Torque", vis.rgba.contacttorque);
    ImGui::ColorEdit4("Contact Gap", vis.rgba.contactgap);
    ImGui::ColorEdit4("Range Finder", vis.rgba.rangefinder);
    ImGui::ColorEdit4("Constraint", vis.rgba.constraint);
    ImGui::ColorEdit4("Slider Crank", vis.rgba.slidercrank);
    ImGui::ColorEdit4("Crank Broken", vis.rgba.crankbroken);
    ImGui::ColorEdit4("Frustum", vis.rgba.frustum);
    ImGui::ColorEdit4("Bounding Vol.", vis.rgba.bv);
    ImGui::ColorEdit4("BV Active", vis.rgba.bvactive);
    ImGui::TreePop();
  }

  ImGui::PopItemWidth();
}

void RenderingGui(const mjModel* model, mjvOption* vis_options,
                  mjtByte* render_flags, float min_width) {
  const float available_width =
      ImGui::GetContentRegionAvail().x - ImGui::GetTreeNodeToLabelSpacing();
  const int num_cols = std::clamp(
      static_cast<int>(std::floor(available_width / min_width)), 1, 6);

  if (ImGui::TreeNodeEx("Model Elements", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Unindent(ImGui::GetTreeNodeToLabelSpacing() / 2);

    if (ImGui::BeginTable("##ModelElementsTable", num_cols)) {
      const ImVec2 size = GetFlexElementSize(num_cols);
      for (int i = 0; i < mjNVISFLAG; ++i) {
        ImGui::TableNextColumn();
        ImGui_ButtonToggle(mjVISSTRING[i][0], &vis_options->flags[i], size);
      }
      ImGui::EndTable();
    }

    ImGui::Indent(ImGui::GetTreeNodeToLabelSpacing() / 2);
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Render Flags", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Unindent(ImGui::GetTreeNodeToLabelSpacing() / 2);

    if (ImGui::BeginTable("##RenderFlagsTable", num_cols)) {
      const ImVec2 size = GetFlexElementSize(num_cols);
      for (int i = 0; i < mjNRNDFLAG; ++i) {
        ImGui::TableNextColumn();
        ImGui_ButtonToggle(mjRNDSTRING[i][0], &render_flags[i], size);
      }
      ImGui::EndTable();
    }

    ImGui::Indent(ImGui::GetTreeNodeToLabelSpacing() / 2);
    ImGui::TreePop();
  }
}

void GroupsGui(const mjModel* model, mjvOption* vis_options, float min_width) {
  const float available_width = ImGui::GetContentRegionAvail().x;
  // We limit the number of columns to 1, 2, 3, or 6 depending on how much
  // space the window has available.
  int num_cols = std::clamp(
      static_cast<int>(std::floor(available_width / min_width)), 1, 6);
  if (num_cols == 4 || num_cols == 5) {
    num_cols = 3;
  }

  auto GroupGui = [&](const char* name, mjtByte* group) {
    if (ImGui::TreeNodeEx(name, ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::Unindent(ImGui::GetTreeNodeToLabelSpacing() / 2);

      char label[64];
      std::snprintf(label, sizeof(label), "##%s", name);
      if (ImGui::BeginTable(label, num_cols)) {
        const ImVec2 size = GetFlexElementSize(num_cols);
        for (int i = 0; i < 6; ++i) {
          ImGui::TableNextColumn();
          std::snprintf(label, sizeof(label), "%s %d", name, i);
          ImGui_ButtonToggle(label, &group[i], size);
        }

        ImGui::EndTable();
      }

      ImGui::Indent(ImGui::GetTreeNodeToLabelSpacing() / 2);
      ImGui::TreePop();
    }
  };

  GroupGui("Geoms", vis_options->geomgroup);
  GroupGui("Sites", vis_options->sitegroup);
  GroupGui("Joints", vis_options->jointgroup);
  GroupGui("Tendons", vis_options->tendongroup);
  GroupGui("Actuators", vis_options->actuatorgroup);
  GroupGui("Flexes", vis_options->flexgroup);
  GroupGui("Skins", vis_options->skingroup);
}

void NoiseGui(const mjModel* model, const mjData* data, float& noise_scale,
              float& noise_rate) {
  const float item_width = ImGui::GetWindowWidth() * .6f;
  ImGui::PushItemWidth(item_width);
  ImGui::SliderFloat("Scale", &noise_scale, 0, 1);
  ImGui::SliderFloat("Rate", &noise_rate, 0, 4);
  ImGui::PopItemWidth();
}

void JointsGui(const mjModel* model, const mjData* data,
               const mjvOption* vis_options) {
  const float item_width = ImGui::GetWindowWidth() * .6f;
  ImGui::PushItemWidth(item_width);

  char name[100];
  for (int i = 0; i < model->njnt; ++i) {
    if (model->jnt_type[i] != mjJNT_HINGE &&
        model->jnt_type[i] != mjJNT_SLIDE) {
      continue;
    }
    const int group = std::clamp(model->jnt_group[i], 0, mjNGROUP - 1);
    if (!vis_options->jointgroup[group]) {
      continue;
    }

    const char* jnt_name = model->names + model->name_jntadr[i];
    if (*jnt_name) {
      std::snprintf(name, sizeof(name), "%s", jnt_name);
    } else {
      std::snprintf(name, sizeof(name), "joint %d", i);
    }

    double min = -1.0;
    double max = 1.0;
    if (model->jnt_limited[i]) {
      min = model->jnt_range[2 * i + 0];
      max = model->jnt_range[2 * i + 1];
    } else if (model->jnt_type[i] == mjJNT_SLIDE) {
      min = -1.0;
      max = 1.0;
    } else {
      min = -3.1416;
      max = 3.1416;
    }

    const int data_adr = model->jnt_qposadr[i];
    ImGui_Slider(name, &data->qpos[data_adr], min, max);
  }

  ImGui::PopItemWidth();
}

void ControlsGui(const mjModel* model, const mjData* data,
                 const mjvOption* vis_options) {
  const float item_width = ImGui::GetWindowWidth() * .6f;
  ImGui::PushItemWidth(item_width);

  if (ImGui::Button("Clear All")) {
    mju_zero(data->ctrl, model->nu);
  }

  char name[100];
  for (int i = 0; i < model->nu; i++) {
    int group = std::clamp(model->actuator_group[i], 0, mjNGROUP - 1);
    if (!vis_options->actuatorgroup[group]) {
      continue;
    }
    if (group >= 0 && group <= 30 &&
        model->opt.disableactuator & (1 << group)) {
      continue;
    }

    const char* ctrl_name = model->names + model->name_actuatoradr[i];
    if (*ctrl_name) {
      std::snprintf(name, sizeof(name), "%s", ctrl_name);
    } else {
      std::snprintf(name, sizeof(name), "control %d", i);
    }

    double min = -1.0;
    double max = 1.0;
    if (!model->actuator_ctrllimited[i]) {
      min = model->actuator_ctrlrange[2 * i + 0];
      max = model->actuator_ctrlrange[2 * i + 1];
    }
    ImGui_Slider(name, &data->ctrl[i], min, max);
  }

  ImGui::PopItemWidth();
}

void ConvergenceGui(const mjModel* model, mjData* data) {
  if (ImPlot::BeginPlot("Convergence (log 10)", ImVec2(-1, 0),
                        ImPlotFlags_NoMouseText)) {
    ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 2.0f);
    ImPlot::SetupAxis(ImAxis_X1, "iteration", ImPlotAxisFlags_AutoFit);
    ImPlot::SetupAxisLimits(ImAxis_X1, 0, 20, ImPlotCond_Always);
    ImPlot::SetupAxisFormat(ImAxis_Y1, "%.1f");
    ImPlot::SetupAxisLimits(ImAxis_Y1, -20, 5, ImPlotCond_Always);
    ImPlot::SetupLegend(ImPlotLocation_NorthEast);
    ImPlot::SetupFinish();

    const int nisland = data->nefc ? mjMAX(1, mjMIN(data->nisland, mjNISLAND)) : 0;
    for (int k = 0; k < nisland; k++) {
      mjSolverStat* stats = data->solver + k * mjNSOLVER;
      const int npoints =
          mjMIN(mjMIN(data->solver_niter[k], mjNSOLVER), mjMAXLINEPNT);

      ImPlot::PlotLineG("improvement", +[](int i, void* user_data) {
        const mjSolverStat* stats = static_cast<const mjSolverStat*>(user_data);
        const float x = static_cast<float>(i);
        const float y = mju_log10(mju_max(mjMINVAL, stats[i].improvement));
        return ImPlotPoint{x, y};
      }, stats, npoints);

      if (model->opt.solver == mjSOL_PGS) {
        continue;
      }

      ImPlot::PlotLineG("gradient", +[](int i, void* user_data) {
        const mjSolverStat* stats = static_cast<const mjSolverStat*>(user_data);
        const float x = static_cast<float>(i);
        const float y = mju_log10(mju_max(mjMINVAL, stats[i].gradient));
        return ImPlotPoint{x, y};
      }, stats, npoints);

      ImPlot::PlotLineG("lineslope", +[](int i, void* user_data) {
        const mjSolverStat* stats = static_cast<const mjSolverStat*>(user_data);
        const float x = static_cast<float>(i);
        const float y = mju_log10(mju_max(mjMINVAL, stats[i].lineslope));
        return ImPlotPoint{x, y};
      }, stats, npoints);
    }

    ImPlot::PopStyleVar();
    ImPlot::EndPlot();
  }
}

void CountsGui(const mjModel* model, mjData* data) {
  if (ImPlot::BeginPlot("Counts", ImVec2(-1, 0), ImPlotFlags_NoMouseText)) {
    ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 2.0f);
    ImPlot::SetupAxis(ImAxis_X1, "iteration", ImPlotAxisFlags_AutoFit);
    ImPlot::SetupAxisLimits(ImAxis_X1, 0, 20, ImPlotCond_Always);
    ImPlot::SetupAxisFormat(ImAxis_Y1, "%.0f");
    ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 80, ImPlotCond_Always);
    ImPlot::SetupLegend(ImPlotLocation_NorthEast);
    ImPlot::SetupFinish();

    const int nisland = data->nefc ? mjMAX(1, mjMIN(data->nisland, mjNISLAND)) : 0;
    for (int k = 0; k < nisland; k++) {
      const int npoints =
          mjMIN(mjMIN(data->solver_niter[k], mjNSOLVER), mjMAXLINEPNT);

      mjSolverStat* stats = data->solver + k*mjNSOLVER;

      int nefc = nisland == 1 ? data->nefc : data->island_nefc[k];

      ImPlot::PlotLineG("total", +[](int i, void* user_data) {
        const float x = static_cast<float>(i);
        const float y = *(static_cast<int*>(user_data));
        return ImPlotPoint{x, y};
      }, &nefc, npoints);

      ImPlot::PlotLineG("active", +[](int i, void* user_data) {
        const mjSolverStat* stats = static_cast<const mjSolverStat*>(user_data);
        const float x = static_cast<float>(i);
        const float y = stats[i].nactive;
        return ImPlotPoint{x, y};
      }, stats, npoints);

      ImPlot::PlotLineG("changed", +[](int i, void* user_data) {
        const mjSolverStat* stats = static_cast<const mjSolverStat*>(user_data);
        const float x = static_cast<float>(i);
        const float y = stats[i].nchange;
        return ImPlotPoint{x, y};
      }, stats, npoints);

      if (model->opt.solver == mjSOL_PGS) {
        continue;
      }

      ImPlot::PlotLineG("evals", +[](int i, void* user_data) {
        const mjSolverStat* stats = static_cast<const mjSolverStat*>(user_data);
        const float x = static_cast<float>(i);
        const float y = stats[i].neval;
        return ImPlotPoint{x, y};
      }, stats, npoints);

      if (model->opt.solver == mjSOL_CG) {
        continue;
      }

      ImPlot::PlotLineG("updates", +[](int i, void* user_data) {
        const mjSolverStat* stats = static_cast<const mjSolverStat*>(user_data);
        const float x = static_cast<float>(i);
        const float y = stats[i].nupdate;
        return ImPlotPoint{x, y};
      }, stats, npoints);
    }

    ImPlot::PopStyleVar();
    ImPlot::EndPlot();
  }
}

void StatsGui(const mjModel* model, const mjData* data, bool paused,
              float fps) {
  const int num_islands = std::clamp(data->nisland, 1, mjNISLAND);

  // compute solver error (maximum over islands)
  mjtNum solver_err = 0;
  int solver_iter = 0;
  for (int i = 0; i < num_islands; i++) {
    solver_iter += data->solver_niter[i];

    mjtNum solerr_i = 0;
    if (data->solver_niter[i]) {
      const int ind = mjMIN(data->solver_niter[i], mjNSOLVER) - 1;
      const mjSolverStat* stat = data->solver + i * mjNSOLVER + ind;
      solerr_i = mju_min(stat->improvement, stat->gradient);
      if (solerr_i == 0) {
        solerr_i = mju_max(stat->improvement, stat->gradient);
      }
    }
    solver_err = mju_max(solver_err, solerr_i);
  }
  solver_err = mju_log10(mju_max(mjMINVAL, solver_err));

  auto type = paused ? mjTIMER_FORWARD : mjTIMER_STEP;
  auto cpu = data->timer[type].duration / mjMAX(1, data->timer[type].number);
  auto mempct = 100 * data->maxuse_arena / (double)(data->narena);
  auto memlimit = mju_writeNumBytes(data->narena);

  ImGui::Columns(2);
  ImGui::SetColumnWidth(0, ImGui::GetWindowWidth() * 0.4f);
  ImGui::SetColumnWidth(1, ImGui::GetWindowWidth() * 0.6f);

  ImGui::Text("Time");
  ImGui::Text("Size");
  ImGui::Text("CPU");
  ImGui::Text("Solver");
  ImGui::Text("FPS");
  ImGui::Text("Memory");
  if (model->opt.enableflags & mjENBL_ENERGY) {
    ImGui::Text("Energy");
  }
  if (model->opt.enableflags & mjENBL_FWDINV) {
    ImGui::Text("FwdInv");
  }
  if (!(model->opt.disableflags & mjDSBL_ISLAND)) {
    ImGui::Text("Islands");
  }

  ImGui::NextColumn();
  ImGui::Text("%-9.3f", data->time);
  ImGui::Text("%d (%d con)", data->nefc, data->ncon);
  ImGui::Text("%.3f", cpu);
  ImGui::Text("%.1f (%d it)", solver_err, solver_iter);
  ImGui::Text("%0.1f", fps);
  ImGui::Text("%.1f%% of %s", mempct, memlimit);
  if (model->opt.enableflags & mjENBL_ENERGY) {
    ImGui::Text("%.3f", data->energy[0] + data->energy[1]);
  }
  if (model->opt.enableflags & mjENBL_FWDINV) {
    ImGui::Text("%.1f %.1f",
                mju_log10(mju_max(mjMINVAL, data->solver_fwdinv[0])),
                mju_log10(mju_max(mjMINVAL, data->solver_fwdinv[1])));
  }
  if (!(model->opt.disableflags & mjDSBL_ISLAND)) {
    ImGui::Text("%d", data->nisland);
  }
  ImGui::Columns();
}

void BodyPropertiesGui(const mjModel* model, const mjData* data,
                       mjsElement* element, int id) {
  const mjsBody* body = mjs_asBody(element);

  ImGui::Columns(2);
  ImGui::SetColumnWidth(0, ImGui::GetWindowWidth() * 0.4f);
  ImGui::SetColumnWidth(1, ImGui::GetWindowWidth() * 0.6f);

  std::string name = *mjs_getName(body->element);
  if (name.empty()) {
    name = "(Body " + std::to_string(id) + ")";
  }

  ImGui::Columns(2);
  ImGui::SetColumnWidth(0, ImGui::GetWindowWidth() * 0.3f);
  ImGui::SetColumnWidth(1, ImGui::GetWindowWidth() * 0.7f);

  ImGui::Text("Name");
  ImGui::Text("xpos[0]");
  ImGui::Text("xpos[1]");
  ImGui::Text("xpos[2]");
  ImGui::Text("xquat[0]");
  ImGui::Text("xquat[1]");
  ImGui::Text("xquat[2]");
  ImGui::Text("xquat[3]");
  ImGui::Text("mass");

  ImGui::NextColumn();
  ImGui::Text("%s", name.c_str());
  ImGui::Text("%f", data->xpos[3*id+0]);
  ImGui::Text("%f", data->xpos[3*id+1]);
  ImGui::Text("%f", data->xpos[3*id+2]);
  ImGui::Text("%f", data->xquat[4*id+0]);
  ImGui::Text("%f", data->xquat[4*id+1]);
  ImGui::Text("%f", data->xquat[4*id+2]);
  ImGui::Text("%f", data->xquat[4*id+3]);
  ImGui::Text("%f", model->body_mass[id]);
}

void JointPropertiesGui(const mjModel* model, const mjData* data,
                        mjsElement* element, int id) {
  const mjsJoint* joint = mjs_asJoint(element);

  ImGui::Columns(2);
  ImGui::SetColumnWidth(0, ImGui::GetWindowWidth() * 0.4f);
  ImGui::SetColumnWidth(1, ImGui::GetWindowWidth() * 0.6f);

  std::string name = *mjs_getName(joint->element);
  if (name.empty()) {
    name = "(Joint " + std::to_string(id) + ")";
  }

  ImGui::Columns(2);
  ImGui::SetColumnWidth(0, ImGui::GetWindowWidth() * 0.3f);
  ImGui::SetColumnWidth(1, ImGui::GetWindowWidth() * 0.7f);
  ImGui::Text("Name");

  ImGui::NextColumn();
  ImGui::Text("%s", name.c_str());
}

void SitePropertiesGui(const mjModel* model, const mjData* data,
                      mjsElement* element, int id) {
  const mjsSite* site = mjs_asSite(element);

  ImGui::Columns(2);
  ImGui::SetColumnWidth(0, ImGui::GetWindowWidth() * 0.4f);
  ImGui::SetColumnWidth(1, ImGui::GetWindowWidth() * 0.6f);

  std::string name = *mjs_getName(site->element);
  if (name.empty()) {
    name = "(Joint " + std::to_string(id) + ")";
  }

  ImGui::Columns(2);
  ImGui::SetColumnWidth(0, ImGui::GetWindowWidth() * 0.3f);
  ImGui::SetColumnWidth(1, ImGui::GetWindowWidth() * 0.7f);
  ImGui::Text("Name");
  ImGui::Text("site_xpos[0]");
  ImGui::Text("site_xpos[1]");
  ImGui::Text("site_xpos[2]");
  ImGui::Text("site_xmat[0]");
  ImGui::Text("site_xmat[1]");
  ImGui::Text("site_xmat[2]");
  ImGui::Text("site_xmat[3]");
  ImGui::Text("site_xmat[4]");
  ImGui::Text("site_xmat[5]");
  ImGui::Text("site_xmat[6]");
  ImGui::Text("site_xmat[7]");
  ImGui::Text("site_xmat[8]");

  ImGui::NextColumn();
  ImGui::Text("%s", name.c_str());
  ImGui::Text("%f", data->site_xpos[3*id+0]);
  ImGui::Text("%f", data->site_xpos[3*id+1]);
  ImGui::Text("%f", data->site_xpos[3*id+2]);
  ImGui::Text("%f", data->site_xmat[4*id+0]);
  ImGui::Text("%f", data->site_xmat[4*id+1]);
  ImGui::Text("%f", data->site_xmat[4*id+2]);
  ImGui::Text("%f", data->site_xmat[4*id+3]);
  ImGui::Text("%f", data->site_xmat[4*id+4]);
  ImGui::Text("%f", data->site_xmat[4*id+5]);
  ImGui::Text("%f", data->site_xmat[4*id+6]);
  ImGui::Text("%f", data->site_xmat[4*id+7]);
  ImGui::Text("%f", data->site_xmat[4*id+8]);
}

}  // namespace mujoco::platform
