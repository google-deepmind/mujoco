# Copyright 2022 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
"""Python bindings for the Simulate GUI."""

import atexit
import code
import inspect
import math
import os
import threading
import time
from typing import Callable, Optional

import glfw
import mujoco
from mujoco import _simulate
import numpy as np

if not glfw._glfw:  # pylint: disable=protected-access
  raise RuntimeError('GLFW dynamic library handle is not available')
else:
  _simulate.setglfwdlhandle(glfw._glfw._handle)  # pylint: disable=protected-access

# Logarithmically spaced realtime slow-down coefficients (percent).
PERCENT_REALTIME = (
    100, 80, 66, 50, 40, 33, 25, 20, 16, 13,
    10, 8, 6.6, 5, 4, 3.3, 2.5, 2, 1.6, 1.3,
    1, 0.8, 0.66, 0.5, 0.4, 0.33, 0.25, 0.2, 0.16, 0.13,
    0.1
)

# Maximum time mis-alignment before re-sync.
MAX_SYNC_MISALIGN = 0.1

# Fraction of refresh available for simulation.
SIM_REFRESH_FRACTION = 0.7

CallbackType = Callable[[mujoco.MjModel, mujoco.MjData], None]

Simulate = _simulate.Simulate


def _reload_from_file(simulate: Simulate, filename: str):
  """Loads an MJCF model into the Simulate GUI."""
  try:
    m = mujoco.MjModel.from_xml_path(filename)
  except mujoco.FatalError as e:
    m = None
    simulate.load_error = str(e)

  if m is not None:
    d = mujoco.MjData(m)
    simulate.load(m, d)
    mujoco.mj_forward(m, d)
  else:
    d = None

  return m, d


def _physics_loop(simulate: Simulate,
                  m: Optional[mujoco.MjModel],
                  d: Optional[mujoco.MjData]):
  """Physics loop for the Simulate GUI, to be run in a separate thread."""
  ctrlnoise = None
  if m is not None:
    ctrlnoise = np.zeros((m.nu,))

  # CPU-sim synchronization point.
  synccpu = 0.0
  syncsim = 0.0

  # Run until asked to exit.
  while not simulate.exitrequest:
    if simulate.droploadrequest:
      simulate.droploadrequest = 0
      new_m, new_d = _reload_from_file(simulate, simulate.dropfilename)
      if new_m is not None:
        m = new_m
        d = new_d
        ctrlnoise = np.zeros((m.nu,))

    if simulate.uiloadrequest:
      simulate.uiloadrequest_decrement()
      new_m, new_d = _reload_from_file(simulate, simulate.dropfilename)
      if new_m is not None:
        m = new_m
        d = new_d
        ctrlnoise = np.zeros((m.nu,))

    # Sleep for 1 ms or yield, to let main thread run.
    if simulate.run != 0 and simulate.busywait != 0:
      time.sleep(0)
    else:
      time.sleep(0.001)

    with simulate.lock():
      if m is not None:
        assert d is not None
        if simulate.run:
          # Record CPU time at start of iteration.
          startcpu = glfw.get_time()

          elapsedcpu = startcpu - synccpu
          elapsedsim = d.time - syncsim

          # Inject noise.
          if simulate.ctrlnoisestd != 0.0:
            # Convert rate and scale to discrete time (Ornstein–Uhlenbeck).
            rate = math.exp(-m.opt.timestep / simulate.ctrlnoiserate)
            scale = simulate.ctrlnoisestd * math.sqrt(1 - rate * rate)

            for i in range(m.nu):
              # Update noise.
              ctrlnoise[i] = (
                  rate * ctrlnoise[i] + scale * mujoco.mju_standardNormal(None))

              # Apply noise.
              d.ctrl[i] = ctrlnoise[i]

          # Requested slow-down factor.
          slowdown = 100 / PERCENT_REALTIME[simulate.real_time_index]

          # Misalignment: distance from target sim time > MAX_SYNC_MISALIGN.
          misaligned = abs(elapsedcpu / slowdown -
                           elapsedsim) > MAX_SYNC_MISALIGN

          # Out-of-sync (for any reason): reset sync times, step.
          if (elapsedsim < 0 or elapsedcpu < 0 or synccpu == 0 or misaligned or
              simulate.speed_changed):
            # Re-sync.
            synccpu = startcpu
            syncsim = d.time
            simulate.speed_changed = False

            # Clear old perturbations, apply new.
            d.xfrc_applied[:, :] = 0
            simulate.applyposepertubations(0)  # Move mocap bodies only.
            simulate.applyforceperturbations()

            # Run single step, let next iteration deal with timing.
            mujoco.mj_step(m, d)

          # In-sync: step until ahead of cpu.
          else:
            measured = False
            prevsim = d.time
            refreshtime = SIM_REFRESH_FRACTION / simulate.refresh_rate
            # Step while sim lags behind CPU and within refreshtime.
            while (((d.time - syncsim) * slowdown <
                    (glfw.get_time() - synccpu)) and
                   ((glfw.get_time() - startcpu) < refreshtime)):
              # Measure slowdown before first step.
              if not measured and elapsedsim:
                simulate.measured_slowdown = elapsedcpu / elapsedsim
                measured = True

              # Clear old perturbations, apply new.
              d.xfrc_applied[:, :] = 0
              simulate.applyposepertubations(0)  # Move mocap bodies only.
              simulate.applyforceperturbations()

              # Call mj_step.
              mujoco.mj_step(m, d)

              # Break if reset.
              if d.time < prevsim:
                break
        else:  # simulate.run is False: GUI is paused.
          # Apply pose perturbation.
          simulate.applyposepertubations(1)  # Move mocap and dynamic bodies.

          # Run mj_forward, to update rendering and joint sliders.
          mujoco.mj_forward(m, d)


def launch(model: Optional[mujoco.MjModel] = None,
           data: Optional[mujoco.MjData] = None,
           *,
           run_physics_thread: bool = True) -> None:
  """Launches the Simulate GUI."""
  if model is None and data is not None:
    raise ValueError('mjData is specified but mjModel is not')
  elif model is not None and data is None:
    data = mujoco.MjData(model)

  # The simulate object encapsulates the UI.
  simulate = Simulate()

  # Initialize GLFW.
  if not glfw.init():
    raise mujoco.FatalError('could not initialize GLFW')

  atexit.register(glfw.terminate)

  if run_physics_thread:
    physics_thread = threading.Thread(
        target=_physics_loop, args=(simulate, model, data))
    physics_thread.start()

  # Load the initial model, if one is given.
  if model is not None:
    t = threading.Thread(target=simulate.load, args=(model, data))
    t.start()
    del t

  simulate.renderloop()

  if run_physics_thread:
    physics_thread.join()


def launch_repl(model: mujoco.MjModel, data: mujoco.MjData) -> None:
  """EXPERIMENTAL FEATURE: Launches the Simulate GUI in REPL mode."""
  try:
    import IPython  # pylint: disable=g-import-not-at-top
    has_ipython = True
  except ImportError:
    has_ipython = False

  def start_shell(global_variables):
    if has_ipython and IPython.get_ipython() is not None:
      locals().update(global_variables)
      IPython.embed()
    else:
      code.InteractiveConsole(locals=global_variables).interact()

  repl_thread = threading.Thread(
      target=start_shell, args=(inspect.stack()[1][0].f_globals,))
  repl_thread.start()
  launch(model, data, run_physics_thread=False)
  repl_thread.join()


if __name__ == '__main__':
  from absl import app  # pylint: disable=g-import-not-at-top
  from absl import flags  # pylint: disable=g-import-not-at-top

  _MJCF_PATH = flags.DEFINE_string('mjcf', None, 'Path to MJCF file.')

  def main(argv) -> None:
    del argv
    if _MJCF_PATH.value is not None:
      model = mujoco.MjModel.from_xml_path(os.path.expanduser(_MJCF_PATH.value))
      launch(model)
    else:
      launch()

  app.run(main)
