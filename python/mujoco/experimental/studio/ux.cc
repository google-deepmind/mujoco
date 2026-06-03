// Copyright 2026 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Python bindings for MuJoCo platform UX components.

#include <algorithm>
#include <array>
#include <string>
#include <tuple>
#include <vector>

#include <imgui.h>
#include <mujoco/mujoco.h>
#include "third_party/mujoco/src/experimental/platform/helpers.h"
#include "third_party/mujoco/src/experimental/platform/sim/step_control.h"
#include "third_party/mujoco/src/experimental/platform/ux/gui.h"
#include "third_party/mujoco/src/experimental/platform/ux/interaction.h"
#include "structs.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

struct UxState {
  // Read/edited by step_control_gui
  int speed_index = 0;

  // Read/edited by state_gui
  std::vector<mjtNum> state;
  int state_sig = 0;

  // Read/edited by watch_gui
  char watch_field_name[256] = {0};
  int watch_field_index = 0;

  // Read/edited by noise_gui
  float noise_scale = 0.0f;
  float noise_rate = 0.0f;

  // Read/edited by camera_selection_gui
  int camera_index = mujoco::platform::kTumbleCameraIdx;
};

struct RenderFlags {
  std::array<uint8_t, mjNRNDFLAG> flags = {0};
};

PYBIND11_MODULE(ux, m) {
  py::class_<RenderFlags>(m, "RenderFlags")
      .def(py::init<>())
      .def_readwrite("flags", &RenderFlags::flags);

  m.doc() = "MuJoCo platform UX components.";

  py::enum_<mujoco::platform::GuiTheme>(m, "GuiTheme")
      .value("LIGHT", mujoco::platform::GuiTheme::kLight)
      .value("DARK", mujoco::platform::GuiTheme::kDark)
      .value("CLASSIC", mujoco::platform::GuiTheme::kClassic);

  py::class_<UxState>(m, "UxState")
      .def(py::init<>())
      .def_readwrite("speed_index", &UxState::speed_index)
      .def_readwrite("state", &UxState::state)
      .def_readwrite("state_sig", &UxState::state_sig)
      .def_readwrite("watch_field_index", &UxState::watch_field_index)
      .def_readwrite("noise_scale", &UxState::noise_scale)
      .def_readwrite("noise_rate", &UxState::noise_rate)
      .def_readwrite("camera_index", &UxState::camera_index)
      .def_property(
          "watch_field_name",
          [](const UxState& self) {
            return std::string(self.watch_field_name);
          },
          [](UxState& self, const std::string& val) {
            std::snprintf(self.watch_field_name, sizeof(self.watch_field_name),
                          "%s", val.c_str());
          });

  m.def(
      "setup_theme",
      [](mujoco::platform::GuiTheme theme) {
        mujoco::platform::SetupTheme(theme);
      },
      py::arg("theme"), "Set up Dear ImGui visual theme.");

  m.def(
      "configure_docking_layout",
      []() {
        ImVec4 r = mujoco::platform::ConfigureDockingLayout();
        return std::make_tuple(r.x, r.y, r.z, r.w);
      },
      "Configure the docking layout with Options (left) and Inspector (right) "
      "panes. Returns (x, y, w, h) of the central workspace area.");

  m.def(
      "step_control_gui",
      [](const mujoco::python::MjModelWrapper& model,
         mujoco::platform::StepControl* step_control, UxState& ux_state) {
        mujoco::platform::StepControlGui(model.get(), step_control,
                                         ux_state.speed_index);
      },
      py::arg("model"), py::arg("step_control"), py::arg("ux_state"),
      "Render the simulation stepping control GUI. Modifies "
      "ux_state.speed_index.");

  m.def(
      "theme_select_gui",
      [](mujoco::platform::GuiTheme theme) {
        bool changed = mujoco::platform::ThemeSelectGui(&theme);
        return std::make_tuple(changed, theme);
      },
      py::arg("theme"),
      "Render the GUI theme selector. Returns (changed, theme).");

  m.def(
      "label_selection_gui",
      [](mujoco::python::MjvOptionWrapper& vis_options) {
        return mujoco::platform::LabelSelectionGui(vis_options.get());
      },
      py::arg("vis_options"), "Render the visualization label selection GUI.");

  m.def(
      "frame_selection_gui",
      [](mujoco::python::MjvOptionWrapper& vis_options) {
        return mujoco::platform::FrameSelectionGui(vis_options.get());
      },
      py::arg("vis_options"), "Render the visualization frame selection GUI.");

  m.def(
      "camera_selection_gui",
      [](const mujoco::python::MjModelWrapper& model,
         mujoco::python::MjDataWrapper& data,
         mujoco::python::MjvCameraWrapper& camera, UxState& ux_state) {
        bool changed = mujoco::platform::CameraSelectionGui(
            model.get(), data.get(), *camera.get(), ux_state.camera_index);
        return changed;
      },
      py::arg("model"), py::arg("data"), py::arg("camera"), py::arg("ux_state"),
      "Render the camera selection GUI. Modifies ux_state.camera_index. "
      "Returns true if camera changed.");

  m.def(
      "set_camera",
      [](const mujoco::python::MjModelWrapper& model,
         mujoco::python::MjvCameraWrapper& camera, int request_idx) {
        return mujoco::platform::SetCamera(model.get(), camera.get(), request_idx);
      },
      py::arg("model"), py::arg("camera"), py::arg("request_idx"),
      "Set the camera index and update the camera object.");

  m.def(
      "set_speed_index",
      [](mujoco::platform::StepControl* step_control, UxState& ux_state, int idx) {
        mujoco::platform::SetSpeedIndex(step_control, ux_state.speed_index, idx);
      },
      py::arg("step_control"), py::arg("ux_state"), py::arg("idx"),
      "Set the simulation speed index.");

  m.def(
      "physics_gui",
      [](mujoco::python::MjModelWrapper& model, float min_width) {
        mujoco::platform::PhysicsGui(model.get(), min_width);
      },
      py::arg("model"), py::arg("min_width") = 150.0f,
      "Render the physics settings UI.");

  m.def(
      "rendering_gui",
      [](const mujoco::python::MjModelWrapper& model,
         mujoco::python::MjvOptionWrapper& vis_options,
         RenderFlags& render_flags) {
        mjtByte flags[mjNRNDFLAG] = {0};
        for (int i = 0; i < mjNRNDFLAG; ++i) {
          flags[i] = render_flags.flags[i];
        }
        mujoco::platform::RenderingGui(model.get(), vis_options.get(), flags,
                                       150.0f);
        for (int i = 0; i < mjNRNDFLAG; ++i) {
          render_flags.flags[i] = flags[i];
        }
      },
      py::arg("model"), py::arg("vis_options"), py::arg("render_flags"),
      "Render the rendering settings UI. Modifies render_flags in place.");

  m.def(
      "groups_gui",
      [](const mujoco::python::MjModelWrapper& model,
         mujoco::python::MjvOptionWrapper& vis_options, float min_width) {
        mujoco::platform::GroupsGui(model.get(), vis_options.get(), min_width);
      },
      py::arg("model"), py::arg("vis_options"), py::arg("min_width") = 150.0f,
      "Render the visibility groups UI.");

  m.def(
      "visualization_gui",
      [](mujoco::python::MjModelWrapper& model,
         mujoco::python::MjvOptionWrapper& vis_options,
         mujoco::python::MjvCameraWrapper& camera, float min_width) {
        mujoco::platform::VisualizationGui(model.get(), vis_options.get(),
                                           camera.get(), min_width);
      },
      py::arg("model"), py::arg("vis_options"), py::arg("camera"),
      py::arg("min_width") = 150.0f, "Render the visualization settings UI.");

  m.def(
      "controls_gui",
      [](const mujoco::python::MjModelWrapper& model,
         mujoco::python::MjDataWrapper& data,
         mujoco::python::MjvOptionWrapper& vis_options) {
        mujoco::platform::ControlsGui(model.get(), data.get(),
                                      vis_options.get());
      },
      py::arg("model"), py::arg("data"), py::arg("vis_options"),
      "Render the actuator controls UI.");

  m.def(
      "joints_gui",
      [](const mujoco::python::MjModelWrapper& model,
         mujoco::python::MjDataWrapper& data,
         mujoco::python::MjvOptionWrapper& vis_options) {
        mujoco::platform::JointsGui(model.get(), data.get(), vis_options.get());
      },
      py::arg("model"), py::arg("data"), py::arg("vis_options"),
      "Render the joints UI.");

  m.def(
      "sensor_gui",
      [](const mujoco::python::MjModelWrapper& model,
         mujoco::python::MjDataWrapper& data) {
        mujoco::platform::SensorGui(model.get(), data.get());
      },
      py::arg("model"), py::arg("data"), "Render the sensor data plot.");

  m.def(
      "state_gui",
      [](const mujoco::python::MjModelWrapper& model,
         mujoco::python::MjDataWrapper& data, UxState& ux_state,
         float min_width) {
        mujoco::platform::StateGui(model.get(), data.get(), ux_state.state,
                                   ux_state.state_sig, min_width);
      },
      py::arg("model"), py::arg("data"), py::arg("ux_state"),
      py::arg("min_width") = 150.0f,
      "Render the state UI. Modifies ux_state.state and ux_state.state_sig.");

  m.def(
      "watch_gui",
      [](const mujoco::python::MjModelWrapper& model,
         mujoco::python::MjDataWrapper& data, UxState& ux_state) {
        mujoco::platform::WatchGui(
            model.get(), data.get(), ux_state.watch_field_name,
            sizeof(ux_state.watch_field_name), ux_state.watch_field_index);
      },
      py::arg("model"), py::arg("data"), py::arg("ux_state"),
      "Render the watch UI. Modifies ux_state.watch_field_name and "
      "ux_state.watch_field_index.");

  m.def(
      "noise_gui",
      [](const mujoco::python::MjModelWrapper& model,
         mujoco::python::MjDataWrapper& data, UxState& ux_state) {
        mujoco::platform::NoiseGui(model.get(), data.get(),
                                   ux_state.noise_scale, ux_state.noise_rate);
      },
      py::arg("model"), py::arg("data"), py::arg("ux_state"),
      "Render the noise UI. Modifies ux_state.noise_scale and "
      "ux_state.noise_rate.");

  m.def(
      "convergence_gui",
      [](const mujoco::python::MjModelWrapper& model,
         mujoco::python::MjDataWrapper& data) {
        mujoco::platform::ConvergenceGui(model.get(), data.get());
      },
      py::arg("model"), py::arg("data"),
      "Render the solver convergence chart.");

  m.def(
      "counts_gui",
      [](const mujoco::python::MjModelWrapper& model,
         mujoco::python::MjDataWrapper& data) {
        mujoco::platform::CountsGui(model.get(), data.get());
      },
      py::arg("model"), py::arg("data"), "Render the solver counts chart.");

  m.def(
      "stats_gui",
      [](const mujoco::python::MjModelWrapper& model,
         mujoco::python::MjDataWrapper& data, bool paused, float fps) {
        mujoco::platform::StatsGui(model.get(), data.get(), paused, fps);
      },
      py::arg("model"), py::arg("data"), py::arg("paused"), py::arg("fps"),
      "Render the simulation statistics UI.");

  m.attr("FREE_CAMERA_IDX") = mujoco::platform::kFreeCameraIdx;
  m.attr("TUMBLE_CAMERA_IDX") = mujoco::platform::kTumbleCameraIdx;
  m.attr("TRACKING_CAMERA_IDX") = mujoco::platform::kTrackingCameraIdx;

  py::enum_<mujoco::platform::CameraMotion>(m, "CameraMotion")
      .value("ZOOM", mujoco::platform::CameraMotion::ZOOM)
      .value("ORBIT", mujoco::platform::CameraMotion::ORBIT)
      .value("TRUCK_PEDESTAL", mujoco::platform::CameraMotion::TRUCK_PEDESTAL)
      .value("TRUCK_DOLLY", mujoco::platform::CameraMotion::TRUCK_DOLLY)
      .value("PAN_TILT", mujoco::platform::CameraMotion::PAN_TILT)
      .value("PLANAR_MOVE_H", mujoco::platform::CameraMotion::PLANAR_MOVE_H)
      .value("PLANAR_MOVE_V", mujoco::platform::CameraMotion::PLANAR_MOVE_V)
      .export_values();

  m.def(
      "MoveCamera",
      [](const mujoco::python::MjModelWrapper& model,
         const mujoco::python::MjDataWrapper& data,
         mujoco::python::MjvCameraWrapper& cam,
         mujoco::platform::CameraMotion motion, mjtNum dx, mjtNum dy) {
        mujoco::platform::MoveCamera(model.get(), data.get(), cam.get(), motion,
                                     dx, dy);
      },
      py::arg("model"), py::arg("data"), py::arg("cam"), py::arg("motion"),
      py::arg("dx"), py::arg("dy"), "Moves the given camera.");

  m.def(
      "InitPerturb",
      [](const mujoco::python::MjModelWrapper& model,
         const mujoco::python::MjDataWrapper& data,
         const mujoco::python::MjvCameraWrapper& cam,
         mujoco::python::MjvPerturbWrapper& pert, int active) {
        mujoco::platform::InitPerturb(model.get(), data.get(), cam.get(),
                                      pert.get(),
                                      static_cast<mjtPertBit>(active));
      },
      py::arg("model"), py::arg("data"), py::arg("cam"), py::arg("pert"),
      py::arg("active"), "Initializes mouse perturbation.");

  m.def(
      "MovePerturb",
      [](const mujoco::python::MjModelWrapper& model,
         const mujoco::python::MjDataWrapper& data,
         const mujoco::python::MjvCameraWrapper& cam,
         mujoco::python::MjvPerturbWrapper& pert, int action, mjtNum reldx,
         mjtNum reldy) {
        mujoco::platform::MovePerturb(model.get(), data.get(), cam.get(),
                                      pert.get(), static_cast<mjtMouse>(action),
                                      reldx, reldy);
      },
      py::arg("model"), py::arg("data"), py::arg("cam"), py::arg("pert"),
      py::arg("action"), py::arg("reldx"), py::arg("reldy"),
      "Moves mouse perturbation.");

  py::class_<mujoco::platform::PickResult>(m, "PickResult")
      .def_readwrite("dist", &mujoco::platform::PickResult::dist)
      .def_readwrite("body", &mujoco::platform::PickResult::body)
      .def_readwrite("geom", &mujoco::platform::PickResult::geom)
      .def_readwrite("flex", &mujoco::platform::PickResult::flex)
      .def_readwrite("skin", &mujoco::platform::PickResult::skin)
      .def_property(
          "point",
          [](const mujoco::platform::PickResult& res) {
            return py::make_tuple(res.point[0], res.point[1], res.point[2]);
          },
          [](mujoco::platform::PickResult& res, const py::tuple& t) {
            res.point[0] = t[0].cast<mjtNum>();
            res.point[1] = t[1].cast<mjtNum>();
            res.point[2] = t[2].cast<mjtNum>();
          });

  m.def(
      "Pick",
      [](const mujoco::python::MjModelWrapper& model,
         const mujoco::python::MjDataWrapper& data,
         const mujoco::python::MjvCameraWrapper& cam, float x, float y,
         float aspect_ratio, const mujoco::python::MjvOptionWrapper& opt) {
        return mujoco::platform::Pick(model.get(), data.get(), cam.get(), x, y,
                                      aspect_ratio, opt.get());
      },
      py::arg("model"), py::arg("data"), py::arg("cam"), py::arg("x"),
      py::arg("y"), py::arg("aspect_ratio"), py::arg("opt"),
      "Picks object under cursor.");

  m.def(
      "camera_to_string",
      [](const mujoco::python::MjDataWrapper& data,
         const mujoco::python::MjvCameraWrapper& camera) {
        return mujoco::platform::CameraToString(data.get(), camera.get());
      },
      py::arg("data"), py::arg("camera"),
      "Returns an XML string representation of the camera.");
}
