# Copyright 2026 The Newton Developers
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

"""mjwarp-viewer: load and simulate an MJCF with MuJoCo Warp.

Usage: mjwarp-viewer <mjcf XML path> [flags]

Example:
  mjwarp-viewer benchmarks/humanoid/humanoid.xml -o "opt.solver=cg"
"""

import copy
import enum
import logging
import sys
import time
from typing import Sequence

import mujoco
import mujoco.viewer
import numpy as np
import warp as wp
from absl import app
from absl import flags
from etils import epath

import mujoco.mjx.third_party.mujoco_warp as mjw


class EngineOptions(enum.IntEnum):
  """Engine option."""

  WARP = 0
  C = 1


_ENGINE = flags.DEFINE_enum_class("engine", EngineOptions.WARP, EngineOptions, "Simulation engine")
from mujoco.mjx.third_party.mujoco_warp._src import cli

_VIEWER = flags.DEFINE_enum("viewer", "mujoco", ["mujoco", "viser"], "Viewer backend (mujoco native or mjviser web)")

_VIEWER_GLOBAL_STATE = {"running": True, "step_once": False}


def key_callback(key: int) -> None:
  if key == 32:  # Space bar
    _VIEWER_GLOBAL_STATE["running"] = not _VIEWER_GLOBAL_STATE["running"]
    logging.info("RUNNING = %s", _VIEWER_GLOBAL_STATE["running"])
  elif key == 46:  # period
    _VIEWER_GLOBAL_STATE["step_once"] = True


def _compile_step(m, d):
  print("Compiling physics step...", end="", flush=True)
  start = time.time()
  # capture the whole step function as a CUDA graph
  with wp.ScopedCapture() as capture:
    mjw.step(m, d)
  elapsed = time.time() - start
  print(f"done ({elapsed:0.2g}s).")
  return capture.graph


def _make_warp_step_fn(mjm, m, d, graph, ctrls=None):
  ctrlid = 0
  opt = copy.copy(mjm.opt)

  def step_fn(mjm, mjd):
    nonlocal ctrlid, opt, m, graph
    if ctrls is not None and ctrlid < len(ctrls):
      mjd.ctrl[:] = ctrls[ctrlid]
      ctrlid += 1
    if mjm.opt != opt:
      opt = copy.copy(mjm.opt)
      m = mjw.put_model(mjm)
      graph = _compile_step(m, d) if wp.get_device().is_cuda else None
    wp.copy(d.ctrl, wp.array([mjd.ctrl.astype(np.float32)]))
    wp.copy(d.act, wp.array([mjd.act.astype(np.float32)]))
    wp.copy(d.xfrc_applied, wp.array([mjd.xfrc_applied.astype(np.float32)]))
    wp.copy(d.qpos, wp.array([mjd.qpos.astype(np.float32)]))
    wp.copy(d.qvel, wp.array([mjd.qvel.astype(np.float32)]))
    wp.copy(d.time, wp.array([mjd.time], dtype=wp.float32))
    if graph is None:
      mjw.step(m, d)
    else:
      wp.capture_launch(graph)
      wp.synchronize()
    mjw.get_data_into(mjd, mjm, d)

  return step_fn


def _make_c_step_fn(ctrls=None):
  if ctrls is None:
    return mujoco.mj_step

  ctrlid = 0

  def step_fn(mjm, mjd):
    nonlocal ctrlid
    if ctrlid < len(ctrls):
      mjd.ctrl[:] = ctrls[ctrlid]
      ctrlid += 1
    mujoco.mj_step(mjm, mjd)

  return step_fn


def _run_viser_viewer(mjm, mjd, step_fn):
  from mjviser import Viewer as MjViserViewer

  MjViserViewer(mjm, mjd, step_fn=step_fn).run()


def _run_passive_viewer(mjm, mjd, step_fn):
  with mujoco.viewer.launch_passive(mjm, mjd, key_callback=key_callback) as viewer:
    while True:
      start = time.time()
      if _VIEWER_GLOBAL_STATE["running"] or _VIEWER_GLOBAL_STATE["step_once"]:
        _VIEWER_GLOBAL_STATE["step_once"] = False
        step_fn(mjm, mjd)
      viewer.sync()
      elapsed = time.time() - start
      if elapsed < mjm.opt.timestep:
        time.sleep(mjm.opt.timestep - elapsed)


def _main(argv: Sequence[str]) -> None:
  """Runs viewer app."""
  if len(argv) < 2:
    raise app.UsageError("Missing required input: mjcf path.")
  elif len(argv) > 2:
    raise app.UsageError("Too many command-line arguments.")

  wp.config.quiet = flags.FLAGS["verbosity"].value < 1
  wp.init()

  mjm = cli.load_model(epath.Path(argv[1]))
  m, d, rc, ctrls = cli.init_structs(mjw.step, mjm)

  if _ENGINE.value == EngineOptions.C:
    print(
      f"  nbody: {mjm.nbody} nv: {mjm.nv} ngeom: {mjm.ngeom} nu: {mjm.nu}\n"
      f"  solver: {mujoco.mjtSolver(mjm.opt.solver).name} cone: {mujoco.mjtCone(mjm.opt.cone).name}"
      f" iterations: {mjm.opt.iterations} ls_iterations: {mjm.opt.ls_iterations}\n"
      f"  integrator: {mujoco.mjtIntegrator(mjm.opt.integrator).name}\n"
    )
    print(f"MuJoCo C simulating with dt = {mjm.opt.timestep:.3f}...")
  else:
    wp.set_device(cli.DEVICE.value)

    graph = _compile_step(m, d) if wp.get_device().is_cuda else None
    if graph is None:
      mjw.step(m, d)  # warmup step
      print("Running Warp unoptimized on CPU.")
    broadphase, filter = mjw.BroadphaseType(m.opt.broadphase).name, mjw.BroadphaseFilter(m.opt.broadphase_filter).name
    solver, cone = mjw.SolverType(m.opt.solver).name, mjw.ConeType(m.opt.cone).name
    integrator = mjw.IntegratorType(m.opt.integrator).name
    iterations, ls_iterations = m.opt.iterations, m.opt.ls_iterations
    ls_str = f"{'parallel' if m.opt.ls_parallel else 'iterative'} linesearch iterations: {ls_iterations}"
    print(
      f"  nbody: {m.nbody} nv: {m.nv} ngeom: {m.ngeom} nu: {m.nu} is_sparse: {m.is_sparse}\n"
      f"  broadphase: {broadphase} broadphase_filter: {filter}\n"
      f"  solver: {solver} cone: {cone} iterations: {iterations} {ls_str}\n"
      f"  integrator: {integrator} graph_conditional: {m.opt.graph_conditional}"
    )
    print(f"Data\n  nworld: {d.nworld} nconmax: {int(d.naconmax / d.nworld)} njmax: {d.njmax}\n")
    print(f"MuJoCo Warp simulating with dt = {m.opt.timestep.numpy()[0]:.3f}...")

  if _ENGINE.value == EngineOptions.WARP:
    step_fn = _make_warp_step_fn(mjm, m, d, graph, ctrls)
  else:
    step_fn = _make_c_step_fn(ctrls)

  mjd = mujoco.MjData(mjm)
  mjw.get_data_into(mjd, mjm, d)
  if _VIEWER.value == "viser":
    _run_viser_viewer(mjm, mjd, step_fn)
  else:
    _run_passive_viewer(mjm, mjd, step_fn)


def main():
  # absl flags assumes __main__ is the main running module for printing usage documentation
  # pyproject bin scripts break this assumption, so manually set argv and docstring
  sys.argv[0] = "mujoco_warp.viewer"
  sys.modules["__main__"].__doc__ = __doc__
  # default to single world with no noise
  flags.FLAGS.set_default("nworld", 1)
  app.run(_main)


if __name__ == "__main__":
  main()
