# Copyright 2026 The MuJoCo Metal contributors
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy at https://www.apache.org/licenses/LICENSE-2.0
# Unless required by law or agreed in writing, software is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.
"""Fixed-model pendulum comparison for CPU, hybrid and native Metal modes.

The native mode advances the left state with MPS generalized dynamics, solve,
and semi-implicit Euler. Hybrid mode uses Metal mass/bias and CPU solve/Euler;
CPU mode uses ``mj_step`` on both sides. OpenGL rendering is a host service.
"""

import argparse
import ctypes
import json
from pathlib import Path
import sys
import threading
import time

import mujoco
import numpy as np


class Comparison:
  """Compare one selected backend with an independent CPU MuJoCo state."""

  def __init__(self, mode="metal-hybrid"):
    self.model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).with_name("pendulum.xml"))
    )
    self.mode = mode
    self.stage = None
    self.native = None
    self.initial_qpos = self.model.qpos0[None, :].copy()
    self.initial_qvel = np.zeros((1, self.model.nv), dtype=np.float32)
    root_joint = mujoco.mj_name2id(
        self.model, mujoco.mjtObj.mjOBJ_JOINT, "root"
    )
    self.initial_qvel[0, self.model.jnt_dofadr[root_joint]] = 10
    if mode == "metal-hybrid":
      from mujoco_metal import MetalSmoothDynamics, load_model

      self.stage = MetalSmoothDynamics(load_model(self.model))
    elif mode == "metal":
      from mujoco_metal import MetalSimulation

      self.native = MetalSimulation(
          self.model,
          batch_size=1,
          qpos=self.initial_qpos,
          qvel=self.initial_qvel,
      )
    elif mode != "cpu":
      raise ValueError(mode)
    self.actual = mujoco.MjData(self.model)
    self.reference = mujoco.MjData(self.model)
    self.reset()

  def reset(self, display_lock=None):
    snapshot = None
    if self.native is not None:
      self.native.state.reset(qpos=self.initial_qpos, qvel=self.initial_qvel)
      snapshot = self._native_snapshot()

    def reset_host_data():
      for data in (self.actual, self.reference):
        mujoco.mj_resetData(self.model, data)
        data.joint("root").qvel = 10
      if snapshot is not None:
        self._sync_native_display_state(snapshot)
      # Host forward is display preparation only; native stepping owns state.
      for data in (self.actual, self.reference):
        mujoco.mj_forward(self.model, data)

    if display_lock is None:
      reset_host_data()
    else:
      with display_lock():
        reset_host_data()
    self.max_qpos_error = 0.0
    self.max_qvel_error = 0.0

  def step(self, display_lock=None):
    m, d = self.model, self.actual
    if self.native is not None:
      self.native.step()
      snapshot = self._native_snapshot()
      if display_lock is None:
        self._sync_native_display_state(snapshot)
      else:
        with display_lock():
          self._sync_native_display_state(snapshot)
    elif self.stage is None:
      mujoco.mj_step(m, d)
    else:
      output = self.stage.run(d.qpos[None, :], d.qvel[None, :])
      if any(t.device.type != "mps" for t in output.values()):
        raise RuntimeError(
            "Expected native MPS outputs; CPU fallback forbidden"
        )
      mass = output["mass_matrix"][0].cpu().numpy().astype(np.float64)
      bias = output["qfrc_bias"][0].cpu().numpy().astype(np.float64)
      # Fixed model has no passive/applied/actuator/contact forces.
      acceleration = np.linalg.solve(mass, -bias)
      d.qvel[:] += m.opt.timestep * acceleration
      mujoco.mj_integratePos(m, d.qpos, d.qvel, m.opt.timestep)
      d.time += m.opt.timestep
    mujoco.mj_step(m, self.reference)
    for state in (d, self.reference):
      if not np.all(np.isfinite(state.qpos)) or not np.all(
          np.isfinite(state.qvel)
      ):
        raise RuntimeError("Nonfinite simulation state")
      if np.any(state.warning.number):
        raise RuntimeError("MuJoCo reported a simulation warning")
    self.max_qpos_error = max(
        self.max_qpos_error, float(np.max(np.abs(d.qpos - self.reference.qpos)))
    )
    self.max_qvel_error = max(
        self.max_qvel_error, float(np.max(np.abs(d.qvel - self.reference.qvel)))
    )

  def _native_snapshot(self):
    snapshot = self.native.state.snapshot()
    if np.any(snapshot.status != 0):
      raise RuntimeError(f"Native Metal step failed: {snapshot.status.tolist()}")
    return snapshot

  def _sync_native_display_state(self, snapshot):
    """Copy native state for comparison and optional host rendering only."""
    self.actual.qpos[:] = snapshot.qpos[0]
    self.actual.qvel[:] = snapshot.qvel[0]
    self.actual.time = float(snapshot.time[0])

  def update_poses(self):
    # CPU render preparation only; these accelerations never advance Metal state.
    mujoco.mj_forward(self.model, self.actual)
    mujoco.mj_forward(self.model, self.reference)

  def add_reference(self, scene):
    for i in range(self.model.ngeom):
      if scene.ngeom >= scene.maxgeom:
        raise RuntimeError("Insufficient viewer geometry capacity")
      mujoco.mjv_initGeom(
          scene.geoms[scene.ngeom],
          int(self.model.geom_type[i]),
          self.model.geom_size[i],
          self.reference.geom_xpos[i] + [0.65, 0, 0],
          self.reference.geom_xmat[i],
          self.model.geom_rgba[i],
      )
      scene.ngeom += 1

  def report(self):
    if self.mode == "metal":
      solve_and_integration = "MPS dense solve and semi-implicit Euler"
      physics = "native Metal contact_free_euler_v1"
    elif self.mode == "metal-hybrid":
      solve_and_integration = "CPU NumPy solve and semi-implicit Euler"
      physics = "Metal mass/bias with CPU solve/integration"
    else:
      solve_and_integration = "CPU MuJoCo mj_step"
      physics = "CPU MuJoCo"
    return dict(
        mode=self.mode,
        simulated_seconds=self.actual.time,
        max_qpos_error=self.max_qpos_error,
        max_qvel_error=self.max_qvel_error,
        physics=physics,
        gpu_stages=(
            ["mass_matrix", "qfrc_bias", "acceleration_solve", "integration"]
            if self.native
            else ["mass_matrix", "qfrc_bias"] if self.stage else []
        ),
        solve_and_integration=solve_and_integration,
        rendering="OpenGL",
        native_contact_free_stepping=self.native is not None,
        full_metal_stepping=False,
        full_mujoco_metal_support=False,
    )


def _active_macos_display_count():
  """Return the active CoreGraphics display count without opening a window."""
  core_graphics = ctypes.CDLL(
      "/System/Library/Frameworks/CoreGraphics.framework/CoreGraphics"
  )
  get_displays = core_graphics.CGGetActiveDisplayList
  display_ids = (ctypes.c_uint32 * 16)()
  display_count = ctypes.c_uint32()
  get_displays.argtypes = [
      ctypes.c_uint32,
      ctypes.POINTER(ctypes.c_uint32),
      ctypes.POINTER(ctypes.c_uint32),
  ]
  get_displays.restype = ctypes.c_int32
  error = get_displays(16, display_ids, ctypes.byref(display_count))
  if error:
    raise RuntimeError(f"CoreGraphics display query failed with code {error}")
  return display_count.value


def _check_interactive_display(parser, headless, system=None, display_count=None):
  """Fail before GPU setup when macOS has no display for its GUI viewer."""
  if headless:
    return
  system = sys.platform if system is None else system
  if system != "darwin":
    return
  try:
    count = (
        _active_macos_display_count()
        if display_count is None
        else display_count
    )
  except (AttributeError, OSError, RuntimeError) as error:
    parser.error(f"Unable to query active macOS displays: {error}")
  if count == 0:
    parser.error(
        "No active macOS display; use --headless --check for a display-free run."
    )


def camera():
  cam = mujoco.MjvCamera()
  mujoco.mjv_defaultCamera(cam)
  cam.lookat[:] = [0.325, 0, 0]
  cam.distance = 1.65
  cam.azimuth = 90
  cam.elevation = 0
  return cam


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument(
      "--mode",
      choices=["metal", "metal-hybrid", "cpu"],
      default="metal-hybrid",
  )
  parser.add_argument("--headless", action="store_true")
  parser.add_argument("--steps", type=int, default=200)
  parser.add_argument(
      "--check", action="store_true", help="Check short rollout (<= 200 steps)"
  )
  parser.add_argument(
      "--image",
      type=Path,
      help="Save side-by-side PNG after headless rollout (needs OpenGL and Pillow)",
  )
  parser.add_argument(
      "--viewer-seconds",
      type=float,
      default=0,
      help="Close viewer after this wall time; 0 keeps it open",
  )
  args = parser.parse_args()
  if args.steps < 1 or (args.check and (not args.headless or args.steps > 200)):
    parser.error(
        "Use positive steps; --check requires --headless and <= 200 steps"
    )
  _check_interactive_display(parser, args.headless)
  sim = Comparison(args.mode)
  if args.mode == "metal":
    description = "MPS dynamics/solve/integration; OpenGL display"
  elif args.mode == "metal-hybrid":
    description = "Metal M/bias; CPU solve/integration; OpenGL display"
  else:
    description = "CPU mj_step; OpenGL display"
  print(
      "LEFT: " + args.mode + " (" + description + ") | RIGHT: CPU mj_step",
      flush=True,
  )
  print("Press R to reset both states.", flush=True)
  if args.headless:
    for _ in range(args.steps):
      sim.step()
    print(json.dumps(sim.report(), indent=2))
    if args.check:
      assert sim.max_qpos_error < 1e-3, sim.report()
      assert sim.max_qvel_error < 1e-2, sim.report()
    if args.image:
      from PIL import Image

      sim.update_poses()
      with mujoco.Renderer(sim.model, height=480, width=640) as renderer:
        renderer.update_scene(sim.actual, camera=camera())
        sim.add_reference(renderer.scene)
        Image.fromarray(renderer.render()).save(args.image)
    return
  from mujoco import viewer as mj_viewer

  reset = threading.Event()

  def key_callback(key):
    if key == ord("R"):
      reset.set()

  with mj_viewer.launch_passive(
      sim.model, sim.actual, key_callback=key_callback
  ) as viewer:
    cam = camera()
    viewer.cam.lookat[:] = cam.lookat
    viewer.cam.distance = cam.distance
    viewer.cam.azimuth = cam.azimuth
    viewer.cam.elevation = cam.elevation
    deadline = time.monotonic() + args.viewer_seconds
    while viewer.is_running():
      if args.viewer_seconds > 0 and time.monotonic() >= deadline:
        break
      start = time.monotonic()
      if reset.is_set():
        sim.reset(display_lock=viewer.lock)
        reset.clear()
      # Fixed 10 ms of simulation per displayed frame; never drop physics steps.
      for _ in range(10):
        sim.step(display_lock=viewer.lock)
      with viewer.lock():
        sim.update_poses()
        viewer.user_scn.ngeom = 0
        sim.add_reference(viewer.user_scn)
      viewer.sync()
      time.sleep(max(0, 0.01 - (time.monotonic() - start)))
  print(json.dumps(sim.report(), indent=2))


if __name__ == "__main__":
  main()
