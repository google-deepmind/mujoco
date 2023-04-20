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
"""Interactive GUI viewer for MuJoCo."""

import abc
import atexit
import contextlib
import math
import os
import queue
import sys
import threading
import time
from typing import Callable, Optional, Tuple, Union
import weakref

import glfw
import mujoco
from mujoco import _simulate
import numpy as np

if not glfw._glfw:  # pylint: disable=protected-access
  raise RuntimeError('GLFW dynamic library handle is not available')
else:
  _simulate.set_glfw_dlhandle(glfw._glfw._handle)  # pylint: disable=protected-access

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
LoaderType = Callable[[], Tuple[mujoco.MjModel, mujoco.MjData]]

# Loader function that also returns a file path for the GUI to display.
_LoaderWithPathType = Callable[[], Tuple[mujoco.MjModel, mujoco.MjData, str]]
_InternalLoaderType = Union[LoaderType, _LoaderWithPathType]

_Simulate = _simulate.Simulate


class Handle:
  """A handle for interacting with a MuJoCo viewer."""

  def __init__(
      self,
      sim: _Simulate,
      scn: mujoco.MjvScene,
      cam: mujoco.MjvCamera,
      opt: mujoco.MjvOption,
      pert: mujoco.MjvPerturb,
  ):
    self._sim = weakref.ref(sim)
    self._scn = scn
    self._cam = cam
    self._opt = opt
    self._pert = pert

  @property
  def scn(self):
    return self._scn

  @property
  def cam(self):
    return self._cam

  @property
  def opt(self):
    return self._opt

  @property
  def perturb(self):
    return self._pert

  def close(self):
    sim = self._sim()
    if sim is not None:
      sim.exitrequest = 1

  def is_running(self) -> bool:
    sim = self._sim()
    if sim is not None:
      return sim.exitrequest < 2
    return False

  def lock(self):
    sim = self._sim()
    if sim is not None:
      return sim.lock()
    return contextlib.nullcontext()

  def sync(self):
    sim = self._sim()
    if sim is not None:
      with sim.lock():
        sim.sync()

  def __enter__(self):
    return self

  def __exit__(self, exc_type, exc_val, exc_tb):
    self.close()


# Abstract base dispatcher class for systems that require UI calls to be made
# on a specific thread (e.g. macOS). This is subclassed by system-specific
# Python launcher (mjpython) to implement the required dispatching mechanism.
class _MjPythonBase(metaclass=abc.ABCMeta):

  def launch_on_ui_thread(self, model: mujoco.MjModel, data: mujoco.MjData):
    pass

# When running under mjpython, the launcher initializes this object.
_MJPYTHON: Optional[_MjPythonBase] = None


def _file_loader(path: str) -> _LoaderWithPathType:
  """Loads an MJCF model from file path."""

  def load(path=path) -> Tuple[mujoco.MjModel, mujoco.MjData, str]:
    m = mujoco.MjModel.from_xml_path(path)
    d = mujoco.MjData(m)
    return m, d, path

  return load


def _reload(
    simulate: _Simulate, loader: _InternalLoaderType,
    notify_loaded: Optional[Callable[[], None]] = None
) -> Optional[Tuple[mujoco.MjModel, mujoco.MjData]]:
  """Internal function for reloading a model in the viewer."""
  try:
    load_tuple = loader()
  except Exception as e:  # pylint: disable=broad-except
    simulate.load_error = str(e)
  else:
    m, d = load_tuple[:2]

    # If the loader does not raise an exception then we assume that it
    # successfully created mjModel and mjData. This is specified in the type
    # annotation, but we perform a runtime assertion here as well to prevent
    # possible segmentation faults.
    assert m is not None and d is not None

    path = load_tuple[2] if len(load_tuple) == 3 else ''
    simulate.load(m, d, path)

    if notify_loaded:
      notify_loaded()

    return m, d


def _physics_loop(simulate: _Simulate, loader: Optional[_InternalLoaderType]):
  """Physics loop for the GUI, to be run in a separate thread."""
  m: mujoco.MjModel = None
  d: mujoco.MjData = None
  ctrl_noise = np.array([])
  reload = True

  # CPU-sim synchronization point.
  synccpu = 0.0
  syncsim = 0.0

  # Run until asked to exit.
  while not simulate.exitrequest:
    if simulate.droploadrequest:
      simulate.droploadrequest = 0
      loader = _file_loader(simulate.dropfilename)
      reload = True

    if simulate.uiloadrequest:
      simulate.uiloadrequest_decrement()
      reload = True

    if reload and loader is not None:
      result = _reload(simulate, loader)
      if result is not None:
        m, d = result
        ctrl_noise = np.zeros((m.nu,))

    reload = False

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
          if simulate.ctrl_noise_std != 0.0:
            # Convert rate and scale to discrete time (Ornstein–Uhlenbeck).
            rate = math.exp(-m.opt.timestep /
                            max(simulate.ctrl_noise_rate, mujoco.mjMINVAL))
            scale = simulate.ctrl_noise_std * math.sqrt(1 - rate * rate)

            for i in range(m.nu):
              # Update noise.
              ctrl_noise[i] = (rate * ctrl_noise[i] +
                               scale * mujoco.mju_standardNormal(None))

              # Apply noise.
              d.ctrl[i] = ctrl_noise[i]

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

              # Call mj_step.
              mujoco.mj_step(m, d)

              # Break if reset.
              if d.time < prevsim:
                break
        else:  # simulate.run is False: GUI is paused.

          # Run mj_forward, to update rendering and joint sliders.
          mujoco.mj_forward(m, d)


def _launch_internal(
    model: Optional[mujoco.MjModel] = None,
    data: Optional[mujoco.MjData] = None,
    *,
    run_physics_thread: bool,
    loader: Optional[_InternalLoaderType] = None,
    handle_return: Optional['queue.Queue[Handle]'] = None,
) -> None:
  """Internal API, so that the public API has more readable type annotations."""
  if model is None and data is not None:
    raise ValueError('mjData is specified but mjModel is not')
  elif callable(model) and data is not None:
    raise ValueError(
        'mjData should not be specified when an mjModel loader is used')
  elif loader is not None and model is not None:
    raise ValueError('model and loader are both specified')
  elif run_physics_thread and handle_return is not None:
    raise ValueError('run_physics_thread and handle_return are both specified')

  if loader is None and model is not None:

    def _loader(m=model, d=data) -> Tuple[mujoco.MjModel, mujoco.MjData]:
      if d is None:
        d = mujoco.MjData(m)
      return m, d

    loader = _loader

  if model and not run_physics_thread:
    scn = mujoco.MjvScene(model, _Simulate.MAX_GEOM)
  else:
    scn = mujoco.MjvScene()
  cam = mujoco.MjvCamera()
  opt = mujoco.MjvOption()
  pert = mujoco.MjvPerturb()
  simulate = _Simulate(scn, cam, opt, pert, run_physics_thread)

  # Initialize GLFW if not using mjpython.
  if _MJPYTHON is None:
    if not glfw.init():
      raise mujoco.FatalError('could not initialize GLFW')
    atexit.register(glfw.terminate)

  notify_loaded = None
  if handle_return:
    notify_loaded = (
        lambda: handle_return.put_nowait(Handle(simulate, scn, cam, opt, pert)))

  side_thread = None
  if run_physics_thread:
    side_thread = threading.Thread(
        target=_physics_loop, args=(simulate, loader))
  else:
    side_thread = threading.Thread(
        target=_reload, args=(simulate, loader, notify_loaded))

  def make_exit_requester(simulate):
    def exit_requester():
      simulate.exitrequest = True
    return exit_requester

  exit_requester = make_exit_requester(simulate)
  atexit.register(exit_requester)

  side_thread.start()
  simulate.render_loop()
  atexit.unregister(exit_requester)
  side_thread.join()


def launch(model: Optional[mujoco.MjModel] = None,
           data: Optional[mujoco.MjData] = None,
           *,
           loader: Optional[LoaderType] = None) -> None:
  """Launches the Simulate GUI."""
  _launch_internal(
      model, data, run_physics_thread=True, loader=loader)


def launch_from_path(path: str) -> None:
  """Launches the Simulate GUI from file path."""
  _launch_internal(run_physics_thread=True, loader=_file_loader(path))


def launch_passive(model: mujoco.MjModel, data: mujoco.MjData) -> Handle:
  """Launches a passive Simulate GUI without blocking the running thread."""
  if not isinstance(model, mujoco.MjModel):
    raise ValueError(f'`model` is not a mujoco.MjModel: got {model!r}')
  if not isinstance(data, mujoco.MjData):
    raise ValueError(f'`data` is not a mujoco.MjData: got {data!r}')

  mujoco.mj_forward(model, data)
  handle_return = queue.Queue(1)

  if sys.platform != 'darwin':
    thread = threading.Thread(
        target=_launch_internal,
        args=(model, data),
        kwargs=dict(run_physics_thread=False, handle_return=handle_return),
    )
    thread.daemon = True
    thread.start()
  else:
    if not isinstance(_MJPYTHON, _MjPythonBase):
      raise RuntimeError(
          '`launch_passive` requires that the Python script be run under '
          '`mjpython` on macOS')
    _MJPYTHON.launch_on_ui_thread(model, data, handle_return)

  return handle_return.get()


if __name__ == '__main__':
  from absl import app  # pylint: disable=g-import-not-at-top
  from absl import flags  # pylint: disable=g-import-not-at-top

  _MJCF_PATH = flags.DEFINE_string('mjcf', None, 'Path to MJCF file.')

  def main(argv) -> None:
    del argv
    if _MJCF_PATH.value is not None:
      launch_from_path(os.path.expanduser(_MJCF_PATH.value))
    else:
      launch()

  app.run(main)
