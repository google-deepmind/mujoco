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
"""This script runs a simulation and viewer in separate threads communicating asynchronously.

In this example, we will run the viewer in an independent thread communicating
via thread-safe queues.

You must provide a mjcf model file via the first command-line argument.
"""

import copy
import os
import queue
import sys
import threading

from absl import app as absl_app
from absl import flags as absl_flags
import mujoco
from mujoco.experimental.studio import native_viewer as _viewer
from mujoco.experimental.studio import parser
from mujoco.experimental.studio import sim as _sim
from mujoco.experimental.studio import studio_app
from mujoco.experimental.studio import ux
from mujoco.experimental.studio import viewer_protocol as vp
import numpy as np

from mujoco.experimental.dear_imgui import dear_imgui as imgui

_GFX = absl_flags.DEFINE_enum(
    'gfx', None, vp.GFX_MODES, 'Rendering graphics mode.'
)
_WIDTH = absl_flags.DEFINE_integer('width', 1200, 'Width of the output image.')
_HEIGHT = absl_flags.DEFINE_integer('height', 800, 'Height of the output image')


def view(
    sim_to_view: queue.Queue[vp.SimToView],
    view_to_sim: queue.Queue[vp.ViewToSim],
) -> None:
  """Entry-point for thread that renders the simulation."""
  # Block until the first message (containing the model) arrives.
  first_msg = sim_to_view.get()
  assert (
      first_msg.model is not None
  ), 'First message must contain the MuJoCo model.'

  title = os.path.basename(sys.argv[0])
  xfrc_sig = int(mujoco.mjtState.mjSTATE_XFRC_APPLIED)
  xfrc_size = mujoco.mj_stateSize(first_msg.model, xfrc_sig)
  xfrc_state = np.zeros(xfrc_size, np.float64)

  data = mujoco.MjData(first_msg.model)
  app = studio_app.StudioApp(first_msg.model, data)
  viewer = _viewer.NativeViewer(
      app.model,
      title=title,
      width=_WIDTH.value,
      height=_HEIGHT.value,
      gfx=_GFX.value,
  )

  while viewer.is_running() and app.is_running():

    # Drain the queue, keeping only the latest message.
    msg = None
    while not sim_to_view.empty():
      msg = sim_to_view.get()

    # Update the camera and compute the perturbation.
    app.handle_mouse_events(viewer.camera, viewer.vis_options, viewer.perturb)

    # Sync state from the backend if a new payload actually arrived.
    if msg is not None and msg.state is not None:
      mujoco.mj_setState(app.model, app.data, msg.state, msg.state_sig)
      mujoco.mj_forward(app.model, app.data)

    # Always apply the perturbation forces from the viewer.
    app.apply_perturb(viewer.perturb)

    # Transmit user interaction when we get a new state
    if msg is not None:
      mujoco.mj_getState(app.model, app.data, xfrc_state, xfrc_sig)
      view_to_sim.put(vp.ViewToSim(state=xfrc_state, state_sig=xfrc_sig))

    # Build the UI.
    ux.setup_theme(app.theme)
    if imgui.Begin(
        'Settings',
        flags=int(imgui.WindowFlags.AlwaysAutoResize)
        | int(imgui.WindowFlags.NoTitleBar)
        | int(imgui.WindowFlags.NoCollapse),
    ):
      imgui.PushItemWidth(200.0)

      imgui.SetNextItemWidth(-1)
      if imgui.Button('Reset Simulation'):
        view_to_sim.put(vp.ViewToSim(reset=True))

      imgui.PopItemWidth()
    imgui.End()

    viewer.sync(app.model, app.data)


def sim(
    data: mujoco.MjData,
    model: mujoco.MjModel,
    sim_to_view: queue.Queue[vp.SimToView],
    view_to_sim: queue.Queue[vp.ViewToSim],
    view_thread: threading.Thread,
) -> None:
  """Entry-point for thread that runs the simulation."""

  sim_to_view.put(vp.SimToView(model=copy.copy(model)))

  step_control = _sim.StepControl()
  integration_sig = int(mujoco.mjtState.mjSTATE_INTEGRATION)
  integration_size = mujoco.mj_stateSize(model, integration_sig)
  integration_state = np.empty(integration_size, np.float64)

  msg = vp.ViewToSim()

  while view_thread.is_alive():
    while not view_to_sim.empty():
      msg = view_to_sim.get()

    if msg.reset:
      mujoco.mj_resetData(model, data)
      mujoco.mj_forward(model, data)
      msg.reset = False

    # Apply perturbation forces received from the viewer thread.
    if msg.state is not None:
      mujoco.mj_setState(model, data, msg.state, msg.state_sig)

    # Advance the simulation keeping up with real-time.
    step_control.advance(model, data)

    # Send the simulation state every iteration. The in-process queue is
    # essentially free and the viewer drains to keep only the latest.
    mujoco.mj_getState(model, data, integration_state, integration_sig)
    sim_to_view.put(
        vp.SimToView(state=integration_state, state_sig=integration_sig)
    )


def main(argv: list[str]) -> None:
  if len(argv) < 2:
    print('Usage: async_thread <model_path.xml>')
    sys.exit(1)

  data = parser.parse(argv[1])
  model = data.model

  # Queues for communication between the simulation and viewer threads.
  sim_to_view = queue.Queue()
  view_to_sim = queue.Queue()

  # Start the viewer thread.
  view_thread = threading.Thread(
      target=view, args=(sim_to_view, view_to_sim), name='Viewer'
  )
  view_thread.start()

  # Start the simulation in the main thread.
  sim(data, model, sim_to_view, view_to_sim, view_thread)

  # Wait for the viewer thread to fully exit.
  view_thread.join(timeout=5.0)


if __name__ == '__main__':
  absl_app.run(main)
