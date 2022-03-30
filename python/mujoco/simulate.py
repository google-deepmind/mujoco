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
"""Wrap the pybind11 bound Simulate class to provide type conversions"""

import mujoco
from mujoco import _simulate

import glfw
import numpy as np

import ctypes
import math
import threading
import time

class Simulate(_simulate.Simulate):
  def __init__(self):
    super().__init__()

  def _get_refreshRate(self):
    return self.getrefreshRate()

  refreshRate = property(
    fget=_get_refreshRate)

  def _get_drop_file_name(self):
    return self.getdropfilename()

  dropfilename = property(
        fget=_get_drop_file_name,
        doc="char* from Simulate"
    )

  def _get_file_name(self):
    return self.getfilename()

  filename = property(
        fget=_get_file_name,
        doc="char* from Simulate"
    )

  def _set_loadError(self, loadError):
    self.setloadError(loadError)

  loadError = property(
        fset=_set_loadError,
        doc="set string to Simulate"
    )

  def _get_exit_request(self):
    return self.getexitrequest()

  def _set_exit_request(self, exitrequest):
    self.setexitrequest(exitrequest)

  exitrequest = property(
        fget=_get_exit_request,
        fset=_set_exit_request,
        doc="atomic bool from Simulate")

  def _get_ui_load_request(self):
    return self.getuiloadrequest()

  def _set_ui_load_request(self, uiloadrequest):
    self.setuiloadrequest(uiloadrequest)

  uiloadrequest = property(
        fget=_get_ui_load_request,
        fset=_set_ui_load_request,
        doc="atomic int from Simulate")

  def _get_drop_load_request(self):
    return self.getdroploadrequest()

  def _set_drop_load_request(self, droploadrequest):
    self.setdroploadrequest(droploadrequest)

  droploadrequest = property(
        fget=_get_drop_load_request,
        fset=_set_drop_load_request,
        doc="atomic bool from Simulate")

def load_and_step_model(m, d, simulate, filename, preload_callback=None, load_callback=None):
  if preload_callback is not None:
    preload_callback(m, d) # Call with old model/data so user can do cleanup

  try:
    mnew = mujoco.MjModel.from_xml_path(filename)
    loadError = None
  except Exception as e:
    print('Error loading using from_xml_path')
    print(e)
    mnew = None
    loadError = str(e)

  if mnew is not None:
    dnew = mujoco.MjData(mnew)
    simulate.load(filename, mnew, dnew, False)
    mujoco.mj_forward(mnew, dnew)
  else:
    mnew = None
    dnew = None

  if load_callback is not None:
    load_callback(mnew, dnew, loadError)

  return mnew, dnew, loadError

def run_physics_loop(simulate, preload_callback=None, load_callback=None, file=None):
  # request loadmodel if file given (otherwise drag-and-drop)
  if file is not None:
    m, d, loadError = load_and_step_model(None, None, simulate, file, preload_callback, load_callback)
    ctrlnoise = np.zeros((m.nu,))
    if loadError is not None:
      simulate.loadError = loadError
  else:
    m = None
    d = None
    ctrlnoise = None

  # constants
  syncmisalign = 0.1    # maximum time mis-alignment before re-sync
  refreshfactor = 0.7   # fraction of refresh available for simulation

  # cpu-sim synchronization point
  cpusync = 0.0
  simsync = 0.0

  # run until asked to exit
  while not simulate.exitrequest:
    if simulate.droploadrequest:
      simulate.droploadrequest = 0
      m_, d_, loadError = load_and_step_model(m, d, simulate, simulate.dropfilename,
                                              preload_callback, load_callback)

      if m_ is not None:
        m = m_
        d = d_
        ctrlnoise = np.zeros((m.nu,))
      else:
        simulate.loadError = loadError

    if simulate.uiloadrequest:
      simulate.uiloadrequest_fetch_sub(1)
      m_, d_, loadError = load_and_step_model(m, d, simulate, simulate.filename,
                                              preload_callback, load_callback)
      if m_ is not None:
        m = m_
        d = d_
        ctrlnoise = np.zeros((m.nu,))
      else:
        simulate.loadError = loadError

    # sleep for 1 ms or yield, to let main thread run
    # yield results in busy wait - which has better timing but kills battery life
    if simulate.run != 0 and simulate.busywait != 0:
      time.sleep(0)
    else:
      time.sleep(0.001)

    # Start exclusive access
    simulate.lock()

    # run only if model is present
    if m is not None:
      # running
      if simulate.run != 0:
        # record cpu time at start of iteration
        tmstart = glfw.get_time()

        # inject noise
        if simulate.ctrlnoisestd != 0.0:
          # convert rate and scale to discrete time given current timestep
          rate = math.exp(-m.opt.timestep / simulate.ctrlnoiserate)
          scale = simulate.ctrlnoisestd * math.sqrt(1-rate*rate)

          for i in range(m.nu):
            # update noise
            ctrlnoise[i] = rate * ctrlnoise[i] + scale * mujoco.mju_standardNormal(None)
            # apply noise
            d.ctrl[i] = ctrlnoise[i]

        # out-of-sync (for any reason)
        offset = abs((d.time*simulate.slow_down - simsync) - (tmstart - cpusync))
        if (d.time*simulate.slow_down < simsync or tmstart < cpusync or cpusync == 0.0 or
            offset > syncmisalign*simulate.slow_down or simulate.speed_changed):
          # re-sync
          cpusync = tmstart
          simsync = d.time*simulate.slow_down
          simulate.speed_changed = False

          # clear old perturbations, apply new
          d.xfrc_applied[:, :] = 0
          simulate.applyposepertubations(0) # move mocap bodies only
          simulate.applyforceperturbations()

          # run single step, let next iteration deal with timing
          mujoco.mj_step(m, d)

        # in-sync
        else:
          while ((d.time*simulate.slow_down - simsync) < (glfw.get_time() - cpusync) and
                 (glfw.get_time() - tmstart) < (refreshfactor/simulate.refreshRate)):
            # clear old perturbations, apply new
            d.xfrc_applied[:, :] = 0
            simulate.applyposepertubations(0) # move mocap bodies only
            simulate.applyforceperturbations()

            # run mj_step
            prevtm = d.time*simulate.slow_down
            mujoco.mj_step(m, d)

            # break on reset
            if d.time*simulate.slow_down < prevtm:
              break

      # paused
      else:
        # apply pose perturbation
        simulate.applyposepertubations(1) # move mocap and dynamic bodies

        # run mj_forward, to update rendering and joint sliders
        mujoco.mj_forward(m, d)

    # end exclusive access
    simulate.unlock()

def run_simulate_and_physics(file=None, preload_callback=None, load_callback=None):
  # simulate object encapsulates the UI
  simulate = Simulate()

  # init GLFW
  if not glfw.init():
    raise mujoco.FatalError('could not initialize GLFW')

  # if m is not None:
  physics_thread = threading.Thread(target=lambda: run_physics_loop(simulate, preload_callback=preload_callback, load_callback=load_callback, file=file))
  physics_thread.start()

  # start simulation thread (this creates the UI)
  simulate.renderloop()
  physics_thread.join()

  glfw.terminate()
