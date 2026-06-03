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
"""This script runs a simulation and viewer in separate processes communicating asynchronously.

In this example, we will run the viewer in an independent process communicating
via multiprocessing queues. Controls are provided to simulate network transit
latency and adjust the communication rates.

You must provide a mjcf model file via the first command-line argument.
"""

import dataclasses
import multiprocessing
import os
import sys
import time

from absl import app as absl_app
from absl import flags as absl_flags
import mujoco
from mujoco.experimental.studio import native_viewer as _viewer
from mujoco.experimental.studio import sim as _sim
from mujoco.experimental.studio import studio_app
from mujoco.experimental.studio import ux
import numpy as np

from mujoco.experimental.dear_imgui import dear_imgui as imgui

_GFX = absl_flags.DEFINE_string('gfx', '', 'Rendering graphics mode.')
_WIDTH = absl_flags.DEFINE_integer('width', 1200, 'Width of the output image.')
_HEIGHT = absl_flags.DEFINE_integer('height', 800, 'Height of the output image')


@dataclasses.dataclass
class SimToView:
  """A message sent from the simulation process to the viewer process."""

  model: mujoco.MjModel | None = None
  data: mujoco.MjData | None = None
  state: np.ndarray | None = None
  state_sig: int = 0
  send_time: float = 0.0


@dataclasses.dataclass
class ViewToSim:
  """A message sent from the viewer process to the simulation process."""

  state: np.ndarray | None = None
  state_sig: int = 0
  reset: bool = False
  send_rate: float = 60.0


class Network:
  """Simulated networking parameters."""

  def __init__(self) -> None:
    self.transit_buffer = []
    self.send_rate = 60.0
    self.network_delay = 0.2

  def get_arrived(self, q: multiprocessing.Queue) -> SimToView | None:
    now = time.time()
    while not q.empty():
      self.transit_buffer.append(q.get())
    arrived = None
    while (
        self.transit_buffer
        and now >= self.transit_buffer[0].send_time + self.network_delay
    ):
      arrived = self.transit_buffer.pop(0)
    return arrived


def view(
    sim_to_view: multiprocessing.Queue,
    view_to_sim: multiprocessing.Queue,
) -> None:
  """Entry-point for process that renders the simulation."""
  # Block until the first message (containing the model) arrives.
  msg = sim_to_view.get()
  assert msg.model is not None, 'First message must contain the MuJoCo model.'

  title = os.path.basename(sys.argv[0])
  xfrc_sig = int(mujoco.mjtState.mjSTATE_XFRC_APPLIED)
  xfrc_size = mujoco.mj_stateSize(msg.model, xfrc_sig)
  xfrc_state = np.zeros(xfrc_size, np.float64)

  app = studio_app.StudioApp(msg.model, msg.data)
  network = Network()
  viewer = _viewer.NativeViewer(
      app.model,
      title=title,
      width=_WIDTH.value,
      height=_HEIGHT.value,
      gfx=_GFX.value,
  )

  while viewer.is_running() and app.is_running():

    # Determine which messages have arrived through the simulated network.
    arrived = network.get_arrived(sim_to_view)

    # Update the camera and compute the perturbation.
    app.handle_mouse_events(viewer.camera, viewer.vis_options, viewer.perturb)

    # Sync state from the backend if a new payload actually arrived.
    if arrived is not None and arrived.state is not None:
      mujoco.mj_setState(app.model, app.data, arrived.state, arrived.state_sig)
      mujoco.mj_forward(app.model, app.data)

    # Always apply the perturbation forces from the viewer.
    app.apply_perturb(viewer.perturb)

    # Transmit user interaction when we get a new state
    if arrived is not None:
      mujoco.mj_getState(app.model, app.data, xfrc_state, xfrc_sig)
      view_to_sim.put(
          ViewToSim(
              send_rate=network.send_rate, state=xfrc_state, state_sig=xfrc_sig
          )
      )

    # Build the UI.
    ux.setup_theme(app.theme)
    if imgui.Begin(
        'Settings',
        flags=int(imgui.WindowFlags.AlwaysAutoResize)
        | int(imgui.WindowFlags.NoTitleBar)
        | int(imgui.WindowFlags.NoCollapse),
    ):
      imgui.PushItemWidth(200.0)
      _, network.network_delay = imgui.SliderFloat(
          'Network Latency (s)', network.network_delay, 0.0, 2.0
      )
      updated, network.send_rate = imgui.SliderFloat(
          'Send Rate (Hz)', network.send_rate, 1.0, 120.0
      )
      if updated:
        view_to_sim.put(ViewToSim(send_rate=network.send_rate))

      imgui.SetNextItemWidth(-1)
      if imgui.Button('Reset Simulation'):
        view_to_sim.put(ViewToSim(reset=True, send_rate=network.send_rate))

      imgui.PopItemWidth()
    imgui.End()

    viewer.sync(app.model, app.data)


def sim(
    data: mujoco.MjData,
    model: mujoco.MjModel,
    sim_to_view: multiprocessing.Queue,
    view_to_sim: multiprocessing.Queue,
    view_process: multiprocessing.Process,
) -> None:
  """Entry-point for process that runs the simulation."""

  sim_to_view.put(SimToView(model=model, data=data))

  step_control = _sim.StepControl()
  integration_sig = int(mujoco.mjtState.mjSTATE_INTEGRATION)
  integration_size = mujoco.mj_stateSize(model, integration_sig)
  integration_state = np.empty(integration_size, np.float64)

  msg = ViewToSim()
  last_send_time = time.time()

  while view_process.is_alive():
    while not view_to_sim.empty():
      msg = view_to_sim.get()

    if msg.reset:
      mujoco.mj_resetData(model, data)
      mujoco.mj_forward(model, data)
      msg.reset = False

    # Apply perturbation forces received from the viewer process.
    if msg.state is not None:
      mujoco.mj_setState(model, data, msg.state, msg.state_sig)

    # Advance the simulation keeping up with real-time.
    step_control.advance(model, data)

    # Send the simulation state paced by the requested send_rate.
    now = time.time()
    if now - last_send_time >= 1.0 / max(1.0, msg.send_rate):
      mujoco.mj_getState(model, data, integration_state, integration_sig)
      sim_to_view.put(
          SimToView(
              state=integration_state,
              state_sig=integration_sig,
              send_time=now,
          )
      )
      last_send_time = now


def main(argv: list[str]) -> None:
  app = studio_app.StudioApp.from_argv(argv)

  # Queues for communication between the simulation and viewer processes.
  sim_to_view = multiprocessing.Queue()
  view_to_sim = multiprocessing.Queue()

  # Start the viewer process.
  view_process = multiprocessing.Process(
      target=view, args=(sim_to_view, view_to_sim)
  )
  view_process.start()

  # Start the simulation in the main process.
  sim(app.data, app.model, sim_to_view, view_to_sim, view_process)


if __name__ == '__main__':
  absl_app.run(main)
