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
"""Viewer-agnostic Python implementation of Studio.

Architecture:
  StudioApp owns the simulation state (model, data) and the UI logic.
  Viewer classes (e.g., NativeViewer) own the window (if required), renderer,
  camera, and visualization options.  The viewer never stores references to
  model or data. Instead, the caller passes them each frame via
  viewer.sync(model, data). This ensures the viewer always renders the current
  model, even if StudioApp.load_model_from_file() swaps it.

The class can be used to implement the full Studio application in Python. By
using the more granular member functions it can also build simple apps that only
use a subset of the Studio UI. This configuration is fully dynamic, there is
nothing to configure in advance, you can change your app by changing the
functions that get called each frame. This class is also viewer-agnostic and as
such does not own camera, vis_options or perturb objects (these are provided by
the viewer).

See the sample/ folder for usage examples.
"""

import os
import sys
import typing

import mujoco
from mujoco.experimental.studio import parser
from mujoco.experimental.studio import sim
from mujoco.experimental.studio import studio_app_events as events
from mujoco.experimental.studio import ux
from mujoco.experimental.studio import viewer_protocol
import numpy as np

from mujoco.experimental.dear_imgui import dear_imgui as imgui

# Type alias for a custom physics step function.
StepFn = typing.Callable[[mujoco.MjModel, mujoco.MjData], None]


def load_model_from_file(
    model_path: str,
) -> tuple[mujoco.MjModel, mujoco.MjData] | None:
  """Loads a model and data from a file path."""
  try:
    data = parser.parse(model_path)
    return data.model, data
  except Exception as ex:  # pylint: disable=broad-except
    print(f'Error loading model from {model_path!r}: {ex}')
    return None


class StudioApp:
  """Viewer-agnostic Python implementation of Studio."""

  @classmethod
  def from_argv(cls, argv: list[str]) -> 'StudioApp':
    """Constructs a StudioApp by parsing a model path from command-line args."""
    if len(argv) < 2:
      model = mujoco.MjSpec().compile()
      data = mujoco.MjData(model)
      app = cls(model, data)
      app.step_control.set_pause_state(sim.PauseState.NORMAL_PAUSED)
      return app

    model_path = argv[1]

    res = load_model_from_file(model_path)
    if res is None:
      sys.exit(-1)
    model, data = res

    app = cls(model, data)
    app.model_path = model_path
    return app

  def load_model_from_file(
      self, model_path: str
  ) -> tuple[mujoco.MjModel, mujoco.MjData] | None:
    """Loads a new model from a file, replacing the current model and data."""
    res = load_model_from_file(model_path)
    if res is None:
      self.status = f'Error loading model from {model_path!r}'
      return None

    model, data = res
    self.model = model
    self.data = data
    self.model_path = model_path
    self.step_control = sim.StepControl()
    self.ux_state = ux.UxState()
    self.status = f'Loaded: {os.path.basename(model_path)!r}'

    return model, data

  def __init__(
      self,
      model: mujoco.MjModel,
      data: mujoco.MjData,
  ):
    """Initializes the Studio application."""
    self.model = model
    self.data = data
    self.model_path = ''

    self.step_control = sim.StepControl()
    self.ux_state = ux.UxState()
    self.theme = ux.GuiTheme.LIGHT
    self.show_stats = False
    self.show_solver = False
    self.should_quit = False
    self.status = 'Ready'
    # TODO(matijak): This should be part of the viewer, also making a struct to
    # pass it around with the camera would be convenient.
    self._cam_speed = 0.001

  def handle_vis_options_keyboard_events(
      self,
      vis_options: mujoco.MjvOption,
      is_freecam_wasd: bool,
  ) -> bool:
    """Toggles visualization flags based on keyboard shortcuts.

    Args:
      vis_options: The visualization options to modify.
      is_freecam_wasd: If True, keys Q/E/A/D are reserved for camera movement
        and will not toggle visualization flags.

    Returns:
      True if a key was handled, False otherwise.
    """
    if imgui.GetIO().WantCaptureKeyboard:
      return False

    return events.handle_vis_options_keyboard_events(
        vis_options, is_freecam_wasd
    )

  def handle_step_control_keyboard_events(self) -> bool:
    """Handles keyboard shortcuts for simulation stepping control.

    Returns:
      True if a key was handled, False otherwise.
    """
    if imgui.GetIO().WantCaptureKeyboard:
      return False

    return events.handle_step_control_keyboard_events(
        self.model, self.data, self.step_control, self.ux_state
    )

  def handle_freecam_wasd_keyboard_events(
      self,
      camera: mujoco.MjvCamera,
  ) -> bool:
    """Handles keyboard shortcuts for free camera movement."""
    if imgui.GetIO().WantCaptureKeyboard:
      return False

    handled, self._cam_speed = events.handle_freecam_wasd_keyboard_events(
        self.model, self.data, camera, self._cam_speed
    )
    return handled

  def handle_keyboard_events(
      self,
      camera: mujoco.MjvCamera,
      vis_options: mujoco.MjvOption,
  ) -> bool:
    """Handle keyboard events according to Studio's bindings."""
    if imgui.GetIO().WantCaptureKeyboard:
      return False

    is_freecam_wasd = self.ux_state.camera_index == ux.FREE_CAMERA_IDX
    if events.handle_step_control_keyboard_events(
        self.model, self.data, self.step_control, self.ux_state
    ):
      return True

    if events.handle_camera_select_keyboard_events(
        self.model, camera, self.ux_state
    ):
      return True

    if events.handle_vis_options_keyboard_events(vis_options, is_freecam_wasd):
      return True

    if is_freecam_wasd:
      handled, self._cam_speed = events.handle_freecam_wasd_keyboard_events(
          self.model, self.data, camera, self._cam_speed
      )
      if handled:
        return True

    return False

  def handle_camera_tracking_mouse_events(
      self,
      camera: mujoco.MjvCamera,
      vis_options: mujoco.MjvOption,
  ) -> None:
    """Handles mouse events for camera tracking."""
    if imgui.GetIO().WantCaptureMouse:
      return

    events.handle_camera_tracking_mouse_events(
        self.model, self.data, camera, vis_options, self.ux_state
    )

  def handle_mouse_events(
      self,
      camera: mujoco.MjvCamera,
      vis_options: mujoco.MjvOption,
      perturb: mujoco.MjvPerturb,
  ) -> None:
    """Handles mouse events."""
    if imgui.GetIO().WantCaptureMouse:
      return

    events.handle_mouse_events(
        self.model, self.data, camera, vis_options, perturb, self.ux_state
    )

  def reset_physics(self) -> None:
    """Reset the physics."""
    mujoco.mj_resetData(self.model, self.data)
    mujoco.mj_forward(self.model, self.data)

  def apply_perturb(self, perturb: mujoco.MjvPerturb) -> None:
    """Apply perturbation the model."""
    if self.step_control.get_pause_state() != sim.PauseState.NORMAL_PAUSED:
      sig = mujoco.mjtState.mjSTATE_XFRC_APPLIED.value
      size = mujoco.mj_stateSize(self.model, sig)
      zero_state = np.zeros(size, np.float64)
      mujoco.mj_setState(self.model, self.data, zero_state, sig)
      mujoco.mjv_applyPerturbPose(self.model, self.data, perturb, 0)
      mujoco.mjv_applyPerturbForce(self.model, self.data, perturb)
    else:
      mujoco.mjv_applyPerturbPose(self.model, self.data, perturb, 1)

  def update_physics(
      self,
      perturb: mujoco.MjvPerturb,
      *,
      step_fn: StepFn | None = None,
  ) -> None:
    """Applies the perturbations and advances the physics.

    Args:
      perturb: The MuJoCo perturbation object.
      step_fn: Optional custom physics step function.  When provided, it is
        called instead of ``step_control.advance``.  The function receives
        ``(model, data)`` and should step the simulation in-place.
    """
    self.apply_perturb(perturb)

    if step_fn is not None:
      step_fn(self.model, self.data)
    else:
      advance_status = self.step_control.advance(self.model, self.data)
      if advance_status == sim.StepStatus.AUTO_RESET:
        self.reset_physics()

  def reset_physics_gui(self) -> None:
    """GUI to Reset the physics i.e., the reset button."""
    button_size = imgui.GetFrameHeight()
    square_size = imgui.Vec2(button_size, button_size)
    icon_reset_model = '\uf0e2'  # FontAwesome "undo" icon.
    if imgui.Button(icon_reset_model, square_size):
      self.reset_physics()
    imgui.SetItemTooltip('Reset')

  def is_running(self) -> bool:
    """Returns True if the application should continue running (called by update())."""
    return not self.should_quit

  def update(
      self,
      camera: mujoco.MjvCamera,
      vis_options: mujoco.MjvOption,
      perturb: mujoco.MjvPerturb,
      *,
      drop_file: str = '',
      step_fn: StepFn | None = None,
  ) -> bool:
    """Update the simulation and handle user input.

    Handles mouse input to compute perturbations or camera motion.
    Handles keyboard input e.g., for keybindings or camera motion.
    Applies the perturbations and advances the physics.
    The argument objects are provided by the viewer.

    Args:
      camera: The MuJoCo camera object.
      vis_options: The MuJoCo visualization options.
      perturb: The MuJoCo perturbation object.
      drop_file: Path of a file dropped into the viewer window. If non-empty the
        current model is replaced with the dropped file.
      step_fn: Optional custom physics step function.  When provided, it is
        called instead of ``step_control.advance``.

    Returns:
      Whether the application should continue running, this is a
      convenience to allow this function to be used in a while loop.
    """
    if drop_file:
      self.load_model_from_file(drop_file)

    self.handle_mouse_events(camera, vis_options, perturb)
    self.handle_keyboard_events(camera, vis_options)
    self.update_physics(perturb, step_fn=step_fn)
    return self.is_running()

  def update_from_viewer(
      self,
      viewer: viewer_protocol.Viewer,
      *,
      step_fn: StepFn | None = None,
  ) -> bool:
    """Convenience wrapper around update() that unpacks viewer attributes.

    Args:
      viewer: A viewer conforming to the Viewer protocol.
      step_fn: Optional custom physics step function.  When provided, it is
        called instead of ``step_control.advance``.

    Returns:
      Whether the application should continue running.
    """
    return self.update(
        viewer.camera,
        viewer.vis_options,
        viewer.perturb,
        drop_file=viewer.get_drop_file(),
        step_fn=step_fn,
    )

  def build_gui(
      self,
      camera: mujoco.MjvCamera,
      vis_options: mujoco.MjvOption,
      render_flags: ux.RenderFlags,
  ) -> None:
    """Emit full Studio UI."""
    ux.setup_theme(self.theme)
    ux.configure_docking_layout()

    # -- Main menu bar --------------------------------------------------------
    if imgui.BeginMainMenuBar():
      if imgui.BeginMenu('File'):
        if imgui.MenuItem('Quit'):
          self.should_quit = True
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
        ux.step_control_gui(self.model, self.step_control, self.ux_state)

        imgui.TableNextColumn()
        ux.camera_selection_gui(self.model, self.data, camera, self.ux_state)

        imgui.SameLine()
        ux.label_selection_gui(vis_options)

        imgui.SameLine()
        ux.frame_selection_gui(vis_options)

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
      ux.rendering_gui(self.model, vis_options, render_flags)
      imgui.TreePop()
    if imgui.TreeNodeEx('Visibility Groups', node_flags):
      ux.groups_gui(self.model, vis_options)
      imgui.TreePop()
    if imgui.TreeNodeEx('Visualization', node_flags):
      ux.visualization_gui(self.model, vis_options, camera)
      imgui.TreePop()
    imgui.End()

    # -- Right pane: Inspector ------------------------------------------------
    imgui.Begin('Inspector')
    if imgui.TreeNodeEx('Noise', node_flags):
      ux.noise_gui(self.model, self.data, self.ux_state)
      imgui.TreePop()
    if imgui.TreeNodeEx('Joints', node_flags):
      ux.joints_gui(self.model, self.data, vis_options)
      imgui.TreePop()
    if imgui.TreeNodeEx('Controls', node_flags):
      ux.controls_gui(self.model, self.data, vis_options)
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
      paused = self.step_control.get_pause_state() != sim.PauseState.UNPAUSED
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
