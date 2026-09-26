# Copyright 2026 The MuJoCo Metal contributors
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy at https://www.apache.org/licenses/LICENSE-2.0
# Unless required by law or agreed in writing, software is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.
"""Fixed-model Metal/CPU pendulum comparison, not a general simulation API.

Left: Metal mass/bias, CPU solve and Euler integration. Right: CPU mj_step.
Rendering uses MuJoCo's OpenGL viewer. No CPU fallback in metal-hybrid mode.
"""

import argparse
import json
from pathlib import Path
import threading
import time

import mujoco
import numpy as np


class Comparison:
  """Advance two independent states of the bundled contact-free model."""

  def __init__(self, mode="metal-hybrid"):
    self.model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).with_name("pendulum.xml"))
    )
    self.mode = mode
    self.stage = None
    if mode == "metal-hybrid":
      from mujoco_metal import MetalSmoothDynamics, load_model

      self.stage = MetalSmoothDynamics(load_model(self.model))
    elif mode != "cpu":
      raise ValueError(mode)
    self.actual = mujoco.MjData(self.model)
    self.reference = mujoco.MjData(self.model)
    self.reset()

  def reset(self):
    for data in (self.actual, self.reference):
      mujoco.mj_resetData(self.model, data)
      data.joint("root").qvel = 10
      mujoco.mj_forward(self.model, data)
    self.max_qpos_error = 0.0
    self.max_qvel_error = 0.0

  def step(self):
    m, d = self.model, self.actual
    if self.stage is None:
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
    return dict(
        mode=self.mode,
        simulated_seconds=self.actual.time,
        max_qpos_error=self.max_qpos_error,
        max_qvel_error=self.max_qvel_error,
        gpu_stages=["mass_matrix", "qfrc_bias"] if self.stage else [],
        solve_and_integration="CPU",
        rendering="OpenGL",
        full_metal_stepping=False,
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
      "--mode", choices=["metal-hybrid", "cpu"], default="metal-hybrid"
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
  sim = Comparison(args.mode)
  print("LEFT: " + args.mode + " | RIGHT: CPU mj_step reference", flush=True)
  print(
      "Metal mode: GPU M/bias; CPU solve/integration; OpenGL display. R resets.",
      flush=True,
  )
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
        sim.reset()
        reset.clear()
      # Fixed 10 ms of simulation per displayed frame; never drop physics steps.
      for _ in range(10):
        sim.step()
      sim.update_poses()
      with viewer.lock():
        viewer.user_scn.ngeom = 0
        sim.add_reference(viewer.user_scn)
      viewer.sync()
      time.sleep(max(0, 0.01 - (time.monotonic() - start)))
  print(json.dumps(sim.report(), indent=2))


if __name__ == "__main__":
  main()
