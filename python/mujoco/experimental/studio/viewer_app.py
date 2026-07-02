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
from typing import Any

import mujoco
from mujoco.experimental.studio import endpoints
from mujoco.experimental.studio import handler_registry
from mujoco.experimental.studio import messages
from mujoco.experimental.studio import parser
from mujoco.experimental.studio import sim
from mujoco.experimental.studio import studio_app_events
from mujoco.experimental.studio import ux
from mujoco.experimental.studio import viewer_protocol
import numpy as np

from mujoco.experimental.dear_imgui import dear_imgui as imgui

@dataclasses.dataclass(frozen=True)
class ViewerAppInitEvent(messages.Event):
  """Lifecycle event dispatched once when the ViewerApp is initialised.

  Handlers that need access to the ViewerApp should handle this event
  and cache the reference.
  """

  viewer_app: 'ViewerApp'


@dataclasses.dataclass(frozen=True)
class BuildGuiEvent(messages.Event):
  """Lifecycle event dispatched on every frame on the viewer side to build ImGui elements."""


@dataclasses.dataclass(frozen=True)
class UpdateEvent(messages.Event):
  """Lifecycle event dispatched on every frame on the viewer side before building GUI."""


class ViewerApp:
  """Viewer component of Studio."""

  def __init__(
      self,
      viewer: viewer_protocol.Viewer,
      endpoint: endpoints.ViewerEndpoint,
      *,
      handlers: list[Any] | None = None,
  ) -> None:
    """Initializes the Studio application.

    Args:
      viewer: A viewer display surface conforming to the Viewer protocol.
      endpoint: The viewer endpoint for communication with the sim side.
      handlers: Optional list of handler instances for viewer-side processing,
        which are classes with methods decorated with ``@handler``.
    """
    self.viewer = viewer
    self.model: mujoco.MjModel = mujoco.MjSpec().compile()
    self.data: mujoco.MjData = mujoco.MjData(self.model)
    mujoco.mj_forward(self.model, self.data)
    self._last_model_id: int | None = id(self.model)
    self.model_path: str = ''
    self.endpoint = endpoint

    # Instantiate handlers from user handlers + framework defaults.
    all_handlers: list[Any] = list(handlers or [])
    all_handlers.append(self)
    self._handlers = handler_registry.HandlerRegistry(all_handlers)

    self.step_control_state = sim.StepControl()  # ONLY for state!
    self.ux_state = ux.UxState()
    self.theme = ux.GuiTheme.LIGHT
    self.show_stats = False
    self.show_solver = False
    self.should_quit = False
    self.status = 'Ready'
    # TODO(matijak): This should be part of the viewer, also making a struct to
    # pass it around with the camera would be convenient.
    self._cam_speed = 0.001

    # Dispatch lifecycle event so handlers can cache the app reference.
    self._handlers.dispatch(ViewerAppInitEvent(viewer_app=self))

  def close(self) -> None:
    """Signals the sim to exit and closes the viewer endpoint, releasing resources."""
    if not self.should_quit:
      self.should_quit = True
      try:
        self.endpoint.send_to_sim(messages.ExitEvent())
      except Exception:  # pylint: disable=broad-exception-caught
        pass  # Ignore exceptions, the sim may have already closed.
    self.endpoint.close()
    self.viewer.close()

  def _send_viewer_snapshots(self) -> None:
    """Sends per-frame viewer-to-sim snapshots.

    Called once per viewer frame.  The snapshot channel coalesces values so the
    sim always sees the latest viewer state without accumulating a backlog.
    """
    noise_scale, noise_rate = self.step_control_state.get_noise_parameters()
    self.endpoint.send_to_sim(
        messages.StepControlSnapshot(
            pause_state=self.step_control_state.get_pause_state(),
            speed=self.step_control_state.get_speed(),
            noise_scale=noise_scale,
            noise_rate=noise_rate,
        )
    )
    self.endpoint.send_to_sim(
        messages.MjOptionSnapshot(opt=copy.deepcopy(self.model.opt))
    )

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
              self.model, self.data, self.viewer.camera, self._cam_speed
          )
      )
      if handled:
        self._cam_speed = cam_speed
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
    self.endpoint.send_to_sim(messages.ResetEvent())
    # Discard any pre-reset snapshots so we don't overwrite the reset state.
    self.endpoint.get_sim_snapshots()

  def apply_perturb(self) -> None:
    """Apply perturbation the model."""
    perturb = self.viewer.perturb
    if (
        self.step_control_state.get_pause_state()
        != sim.PauseState.NORMAL_PAUSED
    ):
      sig = mujoco.mjtState.mjSTATE_XFRC_APPLIED.value
      size = mujoco.mj_stateSize(self.model, sig)
      zero_state = np.zeros(size, np.float64)
      mujoco.mj_setState(self.model, self.data, zero_state, sig)
      mujoco.mjv_applyPerturbPose(self.model, self.data, perturb, 0)
      mujoco.mjv_applyPerturbForce(self.model, self.data, perturb)
    else:
      mujoco.mjv_applyPerturbPose(self.model, self.data, perturb, 1)

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
    return self.viewer.is_running() and not self.should_quit

  def update(self) -> None:
    """Update the simulation and handle user input.

    Handles mouse input to compute perturbations or camera motion.
    Handles keyboard input e.g., for keybindings or camera motion.
    Applies the perturbations and advances the physics.
    """
    drop_file = self.viewer.get_drop_file()
    # Handle file drop: load the model, update local state, notify sim.
    if drop_file:
      try:
        data = parser.parse(drop_file)
        if data is not None:
          self.model, self.data = data.model, data
          self.model_path = drop_file
          self.endpoint.send_to_sim(messages.ModelEvent(model=self.model))
      except Exception as ex:  # pylint: disable=broad-except
        print(f'Error loading model from {drop_file!r}: {ex}')

    # Process incoming events from the simulation.
    incoming_events = self.endpoint.get_sim_events()
    for event in incoming_events:
      self._handlers.dispatch(event)

    # Detect model change from drop_file or ModelEvent (or external swap).
    model_changed = False
    if (
        self._last_model_id is not None
        and id(self.model) != self._last_model_id
    ):
      model_changed = True
      mujoco.mj_forward(self.model, self.data)
      self.step_control_state = sim.StepControl()
      self.ux_state = ux.UxState()
      self._last_model_id = id(self.model)

    # Process incoming snapshots from the simulation process.
    incoming_snapshots = self.endpoint.get_sim_snapshots()
    for snapshot in incoming_snapshots:
      if not model_changed:
        self._handlers.dispatch(snapshot)

    self.handle_mouse_events()
    self.handle_keyboard_events()

    xfrc_sig: int = int(mujoco.mjtState.mjSTATE_XFRC_APPLIED)
    xfrc_size: int = mujoco.mj_stateSize(self.model, xfrc_sig)
    xfrc_state: np.ndarray = np.zeros(xfrc_size, np.float64)

    # Apply perturbation forces from the viewer.
    self.apply_perturb()
    mujoco.mj_getState(self.model, self.data, xfrc_state, xfrc_sig)
    self.endpoint.send_to_sim(
        messages.PerturbEvent(state=xfrc_state, state_sig=xfrc_sig)
    )

    # Send viewer-to-sim snapshots (step control, model options) each frame.
    self._send_viewer_snapshots()

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

  @messages.handler(priority=messages.Priority.INTERNAL)
  def _on_model(self, event: messages.ModelEvent) -> bool:
    self.model = copy.deepcopy(event.model)
    self.data = mujoco.MjData(self.model)
    assert id(self.model) != id(event.model)
    return True

  @messages.handler(priority=messages.Priority.INTERNAL)
  def _on_exit(self, _: messages.ExitEvent) -> bool:
    self.should_quit = True
    self.viewer.close()
    return True

  @messages.handler(priority=messages.Priority.INTERNAL)
  def _on_state(self, event: messages.StateSnapshot) -> bool:
    state_size = mujoco.mj_stateSize(self.model, event.state_sig)
    if len(event.state) == state_size:
      mujoco.mj_setState(
          self.model, self.data, event.state, event.state_sig
      )
      mujoco.mj_forward(self.model, self.data)
    return True

  @messages.handler(priority=messages.Priority.INTERNAL)
  def _on_update(self, _: UpdateEvent) -> None:
    self.update()

  @messages.handler(priority=messages.Priority.INTERNAL)
  def _on_build_gui(self, _: BuildGuiEvent) -> None:
    self.build_gui()


def run_viewer(
    viewer: viewer_protocol.Viewer,
    viewer_endpoint: endpoints.ViewerEndpoint,
    *,
    handlers: list[Any] | None = None,
) -> None:
  """Run the viewer loop with the given viewer and endpoint.

  This is the common viewer-side entry point used by both passive and
  subprocess launchers.

  Runs until the viewer window is closed or the app requests a quit.
  On exit, sends an ExitEvent to the sim side so ViewerHandle.sync() detects
  the shutdown.

  Args:
    viewer: A viewer display surface conforming to the Viewer protocol.
    viewer_endpoint: The viewer endpoint for communication with the sim side.
    handlers: Optional list of handler instances for viewer processing, which
      are classes with methods decorated with ``@handler``.
  """
  # pylint: disable=protected-access
  app = ViewerApp(viewer, viewer_endpoint, handlers=handlers)

  # Viewer main loop.
  while app.is_running():
    app._handlers.dispatch(UpdateEvent())
    app._handlers.dispatch(BuildGuiEvent())
    app.viewer.sync(app.model, app.data)

  app.close()
