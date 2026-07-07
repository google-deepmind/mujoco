# Copyright 2026 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Viewer component of Studio."""

import copy
import dataclasses
import mujoco
from mujoco.experimental.studio import messages
from mujoco.experimental.studio import parser
from mujoco.experimental.studio import sim
from mujoco.experimental.studio import studio_app_events
from mujoco.experimental.studio import ux
from mujoco.experimental.studio import viewer_protocol
from mujoco.experimental.studio import viewer_utils

from mujoco.experimental.dear_imgui import dear_imgui as imgui


@dataclasses.dataclass(frozen=True)
class ViewerAppInitEvent(messages.Event):
  """Lifecycle event dispatched once when the ViewerApp is initialised.

  Handlers that need access to the ViewerApp should handle this event
  and cache the reference.
  """

  viewer_app: 'ViewerApp'


class ViewerApp:
  """ViewerApp wraps a Viewer and adds Studio UI/UX."""

  @property
  def viewer(self) -> viewer_protocol.Viewer:
    assert self._viewer is not None
    return self._viewer

  @viewer.setter
  def viewer(self, value: viewer_protocol.Viewer | None) -> None:
    self._viewer = value

  @property
  def model(self) -> mujoco.MjModel:
    return self.viewer.model

  @model.setter
  def model(self, value: mujoco.MjModel) -> None:
    self.viewer.model = value

  @property
  def model_path(self) -> str:
    return self.viewer.model_path

  @model_path.setter
  def model_path(self, value: str) -> None:
    self.viewer.model_path = value

  @property
  def data(self) -> mujoco.MjData:
    return self.viewer.data

  @data.setter
  def data(self, value: mujoco.MjData) -> None:
    self.viewer.data = value

  def __init__(self) -> None:
    self._viewer: viewer_protocol.Viewer | None = None
    self.theme = ux.GuiTheme.LIGHT
    self.show_stats = False
    self.show_solver = False
    self.status = 'Ready'
    self._reset_app_state()

  @messages.handler(priority=messages.Priority.CRITICAL)
  def _on_viewer_init(self, event: viewer_protocol.ViewerInitEvent) -> None:
    self.viewer = event.viewer
    self.viewer.dispatch(ViewerAppInitEvent(viewer_app=self))

  def _reset_app_state(self) -> None:
    """Resets ViewerApp-specific state (step control, ux)."""
    self.step_control_state = sim.StepControl()
    self.ux_state = ux.UxState()

  def close(self) -> None:
    self.viewer.close()

  def handle_keyboard_events(self) -> None:
    """Handles keyboard events."""

    is_freecam_wasd = self.ux_state.camera_index == ux.FREE_CAMERA_IDX
    if studio_app_events.handle_step_control_keyboard_events(
        self.step_control_state, self.ux_state
    ):
      return

    if studio_app_events.handle_reset_keyboard_events(self.model, self.data):
      self.reset_physics()
      return

    if studio_app_events.handle_camera_select_keyboard_events(
        self.model, self.viewer.camera, self.ux_state
    ):
      return

    if studio_app_events.handle_vis_options_keyboard_events(
        self.viewer.vis_options, is_freecam_wasd
    ):
      return

    if is_freecam_wasd:
      handled, cam_speed = (
          studio_app_events.handle_freecam_wasd_keyboard_events(
              self.model, self.data, self.viewer.camera, self.viewer.cam_speed
          )
      )
      if handled:
        self.viewer.cam_speed = cam_speed
        return

  def handle_camera_tracking_mouse_events(self) -> None:
    """Handles mouse events for camera tracking."""
    return studio_app_events.handle_camera_tracking_mouse_events(
        self.model,
        self.data,
        self.viewer.camera,
        self.viewer.vis_options,
        self.ux_state,
    )

  def handle_mouse_events(
      self,
  ) -> None:
    """Handles mouse events."""
    return studio_app_events.handle_mouse_events(
        self.model,
        self.data,
        self.viewer.camera,
        self.viewer.vis_options,
        self.viewer.perturb,
        self.ux_state,
    )

  def reset_physics(self) -> None:
    """Reset the physics."""
    mujoco.mj_resetData(self.model, self.data)
    mujoco.mj_forward(self.model, self.data)
    self.viewer.send_to_sim(messages.ResetEvent())
    # Discard any pre-reset snapshots so we don't overwrite the reset state.
    self.viewer.get_sim_snapshots()

  def apply_perturb(self) -> None:
    """Apply perturbation the model."""
    is_paused = (
        self.step_control_state.get_pause_state()
        == sim.PauseState.NORMAL_PAUSED
    )
    viewer_utils.apply_perturb(self.viewer, self.model, self.data, is_paused)

  def reset_physics_gui(self) -> None:
    """GUI to Reset the physics i.e., the reset button."""
    button_size = imgui.GetFrameHeight()
    square_size = imgui.Vec2(button_size, button_size)
    icon_reset_model = '\uf0e2'  # FontAwesome "undo" icon.
    if imgui.Button(icon_reset_model, square_size):
      self.reset_physics()
    imgui.SetItemTooltip('Reset')

  def is_running(self) -> bool:
    """Returns True if the application should continue running."""
    return self.viewer.is_running()

  def update(self) -> None:
    """Update the simulation and handle user input.

    Handles mouse input to compute perturbations or camera motion.
    Handles keyboard input e.g., for keybindings or camera motion.
    Applies the perturbations and advances the physics.
    """
    drop_file = self.viewer.get_drop_file()
    # Handle file drop: update viewer model/data, reset app state, notify sim.
    if drop_file:
      try:
        data = parser.parse(drop_file)
        if data is not None:
          # Notify all handlers on the sim side
          self.viewer.send_to_sim(
              messages.ModelEvent(model=data.model, path=drop_file)
          )
          # Notify all handlers on the viewer side.
          self.viewer.dispatch(
              messages.ModelEvent(model=data.model, path=drop_file)
          )
          # Discard all snapshots, including any stale state snapshots
          self.viewer.get_sim_snapshots()
      except Exception as ex:  # pylint: disable=broad-except
        print(f'Error loading model from {drop_file!r}: {ex}')

    # Handle user input.
    self.handle_mouse_events()
    self.handle_keyboard_events()

    # Apply perturbation forces from the viewer.
    self.apply_perturb()

    # Send viewer-to-sim snapshots (step control, model options) each frame.
    noise_scale, noise_rate = self.step_control_state.get_noise_parameters()
    self.viewer.send_to_sim(
        messages.StepControlSnapshot(
            pause_state=self.step_control_state.get_pause_state(),
            speed=self.step_control_state.get_speed(),
            noise_scale=noise_scale,
            noise_rate=noise_rate,
        )
    )
    self.viewer.send_to_sim(
        messages.MjOptionSnapshot(opt=copy.deepcopy(self.model.opt))
    )

  def build_gui(self) -> None:
    """Emit full Studio UI."""
    ux.setup_theme(self.theme)
    ux.configure_docking_layout()

    # -- Main menu bar --------------------------------------------------------
    if imgui.BeginMainMenuBar():
      if imgui.BeginMenu('File'):
        if imgui.MenuItem('Quit'):
          self.close()
        imgui.EndMenu()
      if imgui.BeginMenu('Simulation'):
        imgui.EndMenu()
      if imgui.BeginMenu('Charts'):
        if imgui.MenuItem('Solver', '', self.show_solver):
          self.show_solver = not self.show_solver
        if imgui.MenuItem('Stats', '', self.show_stats):
          self.show_stats = not self.show_stats
        imgui.EndMenu()
      if imgui.BeginMenu('Help'):
        if imgui.MenuItem('Stats', '', self.show_stats):
          self.show_stats = not self.show_stats
        imgui.Separator()
        version = f'Version {mujoco.mj_versionString()}'
        imgui.MenuItem(version)
        imgui.EndMenu()
      imgui.EndMainMenuBar()

    # -- Tool Bar -------------------------------------------------------------
    if imgui.Begin('ToolBar'):
      imgui.PushStyleVar(imgui.StyleVar.CellPadding, imgui.Vec2(0, 0))
      if imgui.BeginTable('##ToolBarTable', 2):
        imgui.TableSetupColumn('', int(imgui.TableColumnFlags.WidthStretch))
        imgui.TableSetupColumn('', int(imgui.TableColumnFlags.WidthFixed))

        imgui.TableNextColumn()
        self.reset_physics_gui()

        imgui.SameLine()
        ux.step_control_gui(self.step_control_state, self.ux_state)

        imgui.TableNextColumn()
        ux.camera_selection_gui(
            self.model,
            self.data,
            self.viewer.camera,
            self.ux_state,
        )

        imgui.SameLine()
        ux.label_selection_gui(self.viewer.vis_options)

        imgui.SameLine()
        ux.frame_selection_gui(self.viewer.vis_options)

        imgui.SameLine()
        changed, self.theme = ux.theme_select_gui(self.theme)
        if changed:
          ux.setup_theme(self.theme)

        imgui.EndTable()
      imgui.PopStyleVar()
    imgui.End()

    # -- Left pane: Options ---------------------------------------------------
    node_flags = int(imgui.TreeNodeFlags.SpanAvailWidth) | int(
        imgui.TreeNodeFlags.Framed
    )

    imgui.Begin('Options')
    if imgui.TreeNodeEx('Physics Settings', node_flags):
      ux.physics_gui(self.model)
      imgui.TreePop()
    if imgui.TreeNodeEx('Rendering Settings', node_flags):
      ux.rendering_gui(
          self.model,
          self.viewer.vis_options,
          self.viewer.render_flags,
      )
      imgui.TreePop()
    if imgui.TreeNodeEx('Visibility Groups', node_flags):
      ux.groups_gui(self.model, self.viewer.vis_options)
      imgui.TreePop()
    if imgui.TreeNodeEx('Visualization', node_flags):
      ux.visualization_gui(
          self.model,
          self.viewer.vis_options,
          self.viewer.camera,
      )
      imgui.TreePop()
    imgui.End()

    # -- Right pane: Inspector ------------------------------------------------
    imgui.Begin('Inspector')
    if imgui.TreeNodeEx('Noise', node_flags):
      ux.noise_gui(self.step_control_state)
      imgui.TreePop()
    if imgui.TreeNodeEx('Joints', node_flags):
      ux.joints_gui(self.model, self.data, self.viewer.vis_options)
      imgui.TreePop()
    if imgui.TreeNodeEx('Controls', node_flags):
      ux.controls_gui(self.model, self.data, self.viewer.vis_options)
      imgui.TreePop()
    if imgui.TreeNodeEx(
        'Sensors', node_flags | int(imgui.TreeNodeFlags.DefaultOpen)
    ):
      ux.sensor_gui(self.model, self.data)
      imgui.TreePop()
    if imgui.TreeNodeEx('Watch', node_flags):
      ux.watch_gui(self.model, self.data, self.ux_state)
      imgui.TreePop()
    if imgui.TreeNodeEx('State', node_flags):
      ux.state_gui(self.model, self.data, self.ux_state)
      imgui.TreePop()
    imgui.End()

    # -- Floating windows -----------------------------------------------------
    if self.show_solver:
      _, self.show_solver = imgui.Begin('Solver', self.show_solver)
      ux.counts_gui(self.model, self.data)
      ux.convergence_gui(self.model, self.data)
      imgui.End()

    if self.show_stats:
      _, self.show_stats = imgui.Begin('Stats', self.show_stats)
      paused = (
          self.step_control_state.get_pause_state() != sim.PauseState.UNPAUSED
      )
      ux.stats_gui(self.model, self.data, paused, 0.0)
      imgui.End()

    # -- Status bar -----------------------------------------------------------
    imgui.PushStyleVar(imgui.StyleVar.CellPadding, imgui.Vec2(0, 0))
    imgui.PushStyleVar(imgui.StyleVar.FramePadding, imgui.Vec2(0, 0))
    imgui.PushStyleVar(imgui.StyleVar.WindowPadding, imgui.Vec2(0, 0))
    if imgui.Begin('StatusBar'):
      imgui.Text(self.status)
    imgui.End()
    imgui.PopStyleVar(3)

  @messages.handler(priority=messages.Priority.CRITICAL)
  def _on_model(self, event: messages.ModelEvent) -> bool:
    del event  # Model/data are owned by the Viewer.
    self._reset_app_state()
    return False  # Do not consume to allow other handlers to receive the event.

  @messages.handler(priority=messages.Priority.INTERNAL)
  def _on_exit(self, _: messages.ExitEvent) -> bool:
    self.close()
    return True

  @messages.handler(priority=messages.Priority.INTERNAL)
  def _on_update(self, _: messages.UpdateEvent) -> None:
    self.update()

  @messages.handler(priority=messages.Priority.INTERNAL)
  def _on_build_gui(self, _: messages.BuildGuiEvent) -> None:
    self.build_gui()
