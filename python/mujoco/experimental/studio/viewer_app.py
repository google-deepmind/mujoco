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
from typing import Protocol

import mujoco
from mujoco.experimental.studio import endpoints
from mujoco.experimental.studio import messages
from mujoco.experimental.studio import parser
from mujoco.experimental.studio import sim
from mujoco.experimental.studio import studio_app_events
from mujoco.experimental.studio import ux
from mujoco.experimental.studio import viewer_protocol
import numpy as np

from mujoco.experimental.dear_imgui import dear_imgui as imgui

# ------------------------------------------------------------------------------
# Viewer application and custom message handlers
# ------------------------------------------------------------------------------


class ViewerEventHandler(Protocol):
  """Invoked once per event received from the simulation."""

  def handle(self, event: messages.Event) -> bool:
    """Return True if event was consumed, False to allow further processing."""
    ...


class ViewerSnapshotHandler(Protocol):
  """Invoked once per snapshot received from the simulation."""

  def handle(self, snapshot: messages.Snapshot) -> bool:
    """Return True if snapshot was consumed, False to allow further processing."""
    ...


class ViewerApp:
  """Viewer component of Studio."""

  def __init__(
      self,
      viewer: viewer_protocol.Viewer,
      endpoint: endpoints.ViewerEndpoint,
      *,
      viewer_event_handler: ViewerEventHandler | None = None,
      viewer_snapshot_handler: ViewerSnapshotHandler | None = None,
  ):
    """Initializes the Studio application."""
    self.viewer = viewer
    self.model: mujoco.MjModel = mujoco.MjSpec().compile()
    self.data: mujoco.MjData = mujoco.MjData(self.model)
    mujoco.mj_forward(self.model, self.data)
    self._last_model_id: int | None = id(self.model)
    self.model_path: str = ''
    self.endpoint = endpoint
    self.viewer_event_handler = viewer_event_handler
    self.viewer_snapshot_handler = viewer_snapshot_handler

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

  def handle_keyboard_events(self):
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
      if (
          self.viewer_event_handler is not None
          and self.viewer_event_handler.handle(event)
      ):
        continue
      if isinstance(event, messages.ModelEvent):
        # A new model was sent (initial load or hot-reload).
        # Deep-copy the incoming model to ensure the viewer operates on its own
        # isolated copy of the C++ struct. This is required if the sim/viewer
        # run in different processes, and means we don't need to lock when
        # running in the same process on different threads.
        self.model = copy.deepcopy(event.model)
        self.data = mujoco.MjData(self.model)

        # Confirm model object is distinct from the incoming event.
        assert id(self.model) != id(event.model)
      elif isinstance(event, messages.ExitEvent):
        self.should_quit = True
        self.viewer.close()

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
      if (
          self.viewer_snapshot_handler is not None
          and self.viewer_snapshot_handler.handle(snapshot)
      ):
        continue
      if not model_changed and isinstance(snapshot, messages.StateSnapshot):
        state_size = mujoco.mj_stateSize(self.model, snapshot.state_sig)
        if len(snapshot.state) == state_size:
          mujoco.mj_setState(
              self.model, self.data, snapshot.state, snapshot.state_sig
          )
          mujoco.mj_forward(self.model, self.data)

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


# ------------------------------------------------------------------------------
# Viewer application run loop and gui/update customization hooks.
# ------------------------------------------------------------------------------


class ViewerUpdateHook(Protocol):
  """Invoked once per viewer frame, after default updates ."""

  def update(self, app: 'ViewerApp') -> None:
    """Update custom state."""
    ...


class ViewerGuiHook(Protocol):
  """Invoked once per viewer frame after all update() calls.

  By deferring GUI hooks until after all updates we ensure custom UI reflects
  the latest application state.
  """

  def build_gui(self, app: 'ViewerApp') -> None:
    """Draw custom UI."""
    ...


def run_viewer(
    viewer: viewer_protocol.Viewer,
    viewer_endpoint: endpoints.ViewerEndpoint,
    *,
    viewer_gui_hook: ViewerGuiHook | None = None,
    viewer_update_hook: ViewerUpdateHook | None = None,
    viewer_event_handler: ViewerEventHandler | None = None,
    viewer_snapshot_handler: ViewerSnapshotHandler | None = None,
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
    viewer_gui_hook: Optional hook to draw custom ImGui panels.
    viewer_update_hook: Optional hook called once per frame before GUI.
    viewer_event_handler: Optional handler for simulation-to-viewer events.
    viewer_snapshot_handler: Optional handler for sim-to-viewer snapshots.
  """
  app = ViewerApp(
      viewer,
      viewer_endpoint,
      viewer_event_handler=viewer_event_handler,
      viewer_snapshot_handler=viewer_snapshot_handler,
  )

  # Viewer main loop.
  while app.is_running():

    # Update the viewer state and handle user input.
    app.update()

    # Update the viewer state.
    if viewer_update_hook is not None:
      viewer_update_hook.update(app)

    # Draw the default Studio GUI.
    app.build_gui()

    # Allow custom GUI elements to be added.
    if viewer_gui_hook is not None:
      viewer_gui_hook.build_gui(app)

    # Sync the viewer display surface with the current model and data.
    app.viewer.sync(app.model, app.data)

  app.close()
