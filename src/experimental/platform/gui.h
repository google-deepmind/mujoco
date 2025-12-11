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

#ifndef MUJOCO_SRC_EXPERIMENTAL_PLATFORM_GUI_H_
#define MUJOCO_SRC_EXPERIMENTAL_PLATFORM_GUI_H_

// A collection of functions for building ImGui panels for common MuJoCo
// visualization and manipulation UX. These functions are primarily used by
// Studio, but are available for other applications.
//
// Like most ImGui functions, the actual "storage" for the GUI state is managed
// by the caller. In most cases, this is already stored in mjModel, mjData,
// mjvOption, etc. But, some functions take additional arguments as needed.

#include <vector>

#include <imgui.h>
#include <mujoco/mujoco.h>

namespace mujoco::platform {

// Standard default UX themes for MuJoCo applications.
enum class GuiTheme {
  kLight,
  kDark,
  kClassic,
};

// Updates the ImGui internal style state to match the requested theme.
void SetupTheme(GuiTheme theme);

// Configures the ImGui docking module to the standard layout used by Studio.
// This includes the following named sections:
//   "ToolBar": fixed size bar spanning the top of the window; for placing
//       buttons and other controls that are always needed.
//   "StatusBar": fixed size bar spanning the bottom of the window; for
//       placing information and controls that are always needed.
//   "Options": resizable section of the left; designed for GUI elements that
//       are used to configure the simulation (e.g. PhysicsGui).
//   "Inspector": resizable section of the right; designed for inspecting
//       or manipulating mjData elements (e.g. ControlsGui).
//   "Explorer": secondary tab connected to the Inspector; designed for
//       displaying the tree of mjSpec elements.
//   "Stats": resizable section below the options; designed for displaying
//       basic simulation statistics (e.g. StatsGui); hidden by default.
//   "Properties": resizable section below the explorer; designed for displaying
//       properties of mjSpec elements (e.g. BodyPropertiesGui); hidden by
//       default.
//
// Returns the size and position of the remaining workspace area which can then
// be used to place additional elements (e.g. floating charts).
ImVec4 ConfigureDockingLayout();

// UX for controlling the physics simulation parameters (e.g. integrator,
// solver, etc.) in mjModel.
void PhysicsGui(mjModel* model, float min_width);

// UX for enabling/disabling visualization groups in mjvOption.
void GroupsGui(const mjModel* model, mjvOption* vis_options, float min_width);

// UX for enabling/disabling rendering (mjtRndFlag) and visualization
// (mjtVisFlag) flags. We combine these into a single function because the sets
// of flags are closely related.
void RenderingGui(const mjModel* model, mjvOption* vis_options,
                  mjtByte* render_flags, float min_width);

// UX for controlling the mjvOption and mjvCamera settings used for visualizing
// scenes (mjvScene).
void VisualizationGui(mjModel* model, mjvOption* vis_options, mjvCamera* camera,
                      float min_width);

// UX for visualizing actuator controls data in mjData.
void ControlsGui(const mjModel* model, const mjData* data,
                 const mjvOption* vis_options);

// UX for visualizing joint data in mjData.
void JointsGui(const mjModel* model, const mjData* data,
               const mjvOption* vis_options);

// UX for visualizing sensor data in mjData.
void SensorGui(const mjModel* model, const mjData* data);

// UX for visualizing the data as returned from mj_getState(). We use a
// user-supplied vector here to avoid allocating memory every frame.
void StateGui(const mjModel* model, mjData* data, std::vector<mjtNum>& state,
              int& state_sig, float min_width);

// UX for visualizing a named field from mjData. `field_name` and `field_index`
// are used to index into the data buffer.
void WatchGui(const mjModel* model, const mjData* data, char* field_name,
              int field_len, int& field_index);

// UX for controlling noise parameters which can then be applied to the
// simulation via StepControl::SetNoiseParameters / StepControl::InjectNoise.
void NoiseGui(const mjModel* model, const mjData* data, float& noise_scale,
              float& noise_rate);

// UX for the solver convergence chart.
void ConvergenceGui(const mjModel* model, mjData* data);

// UX for the solver counts chart.
void CountsGui(const mjModel* model, mjData* data);

// UX for displaying basic simulation information. Note that the pause state and
// FPS needs to be tracked by the caller and passed here to be displayed.
void StatsGui(const mjModel* model, const mjData* data, bool paused, float fps);

// UX for displaying properties of various mjSpec elements.
void BodyPropertiesGui(const mjModel* model, const mjData* data,
                       mjsElement* element, int id);
void JointPropertiesGui(const mjModel* model, const mjData* data,
                        mjsElement* element, int id);
void SitePropertiesGui(const mjModel* model, const mjData* data,
                       mjsElement* element, int id);

}  // namespace mujoco::platform

#endif  // MUJOCO_SRC_EXPERIMENTAL_PLATFORM_GUI_H_
