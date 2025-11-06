# Copyright 2025 The Newton Developers
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
  mjwarp-viewer benchmark/humanoid/humanoid.xml -o "opt.solver=cg"
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

# mjwarp-viewer has priviledged access to a few internal methods
from mujoco.mjx.third_party.mujoco_warp._src.io import find_keys
from mujoco.mjx.third_party.mujoco_warp._src.io import make_trajectory
from mujoco.mjx.third_party.mujoco_warp._src.io import override_model


class EngineOptions(enum.IntEnum):
  """Engine option."""

  WARP = 0
  C = 1


_CLEAR_KERNEL_CACHE = flags.DEFINE_bool("clear_kernel_cache", False, "Clear kernel cache (to calculate full JIT time)")
_ENGINE = flags.DEFINE_enum_class("engine", EngineOptions.WARP, EngineOptions, "Simulation engine")
_NCONMAX = flags.DEFINE_integer("nconmax", None, "Maximum number of contacts.")
_NJMAX = flags.DEFINE_integer("njmax", None, "Maximum number of constraints per world.")
_OVERRIDE = flags.DEFINE_multi_string("override", [], "Model overrides (notation: foo.bar = baz)", short_name="o")
_KEYFRAME = flags.DEFINE_integer("keyframe", 0, "keyframe to initialize simulation.")
_DEVICE = flags.DEFINE_string("device", None, "override the default Warp device")
_REPLAY = flags.DEFINE_string("replay", None, "keyframe sequence to replay, keyframe name must prefix match")

_VIEWER_GLOBAL_STATE = {"running": True, "step_once": False}


def key_callback(key: int) -> None:
  if key == 32:  # Space bar
    _VIEWER_GLOBAL_STATE["running"] = not _VIEWER_GLOBAL_STATE["running"]
    logging.info("RUNNING = %s", _VIEWER_GLOBAL_STATE["running"])
  elif key == 46:  # period
    _VIEWER_GLOBAL_STATE["step_once"] = True


def _load_model(path: epath.Path) -> mujoco.MjModel:
  if not path.exists():
    resource_path = epath.resource_path("mjx") / "third_party/mujoco_warp" / path
    if not resource_path.exists():
      raise FileNotFoundError(f"file not found: {path}\nalso tried: {resource_path}")
    path = resource_path

  print(f"Loading model from: {path}...")
  if path.suffix == ".mjb":
    return mujoco.MjModel.from_binary_path(path.as_posix())

  spec = mujoco.MjSpec.from_file(path.as_posix())
  # check if the file has any mujoco.sdf test plugins
  if any(p.plugin_name.startswith("mujoco.sdf") for p in spec.plugins):
    from mujoco.mjx.third_party.mujoco_warp.test_data.collision_sdf.utils import register_sdf_plugins as register_sdf_plugins

    register_sdf_plugins(mjw)
  return spec.compile()


def _compile_step(m, d):
  print("Compiling physics step...", end="", flush=True)
  start = time.time()
  # capture the whole step function as a CUDA graph
  with wp.ScopedCapture() as capture:
    mjw.step(m, d)
  elapsed = time.time() - start
  print(f"done ({elapsed:0.2g}s).")
  return capture.graph


def _main(argv: Sequence[str]) -> None:
  """Runs viewer app."""
  if len(argv) < 2:
    raise app.UsageError("Missing required input: mjcf path.")
  elif len(argv) > 2:
    raise app.UsageError("Too many command-line arguments.")

  mjm = _load_model(epath.Path(argv[1]))
  mjd = mujoco.MjData(mjm)
  ctrls = None
  ctrlid = 0
  if _REPLAY.value:
    keys = find_keys(mjm, _REPLAY.value)
    if not keys:
      raise app.UsageError(f"Key prefix not find: {_REPLAY.value}")
    ctrls = make_trajectory(mjm, keys)
    mujoco.mj_resetDataKeyframe(mjm, mjd, keys[0])
  elif mjm.nkey > 0 and _KEYFRAME.value > -1:
    mujoco.mj_resetDataKeyframe(mjm, mjd, _KEYFRAME.value)
  mujoco.mj_forward(mjm, mjd)

  if _ENGINE.value == EngineOptions.C:
    override_model(mjm, _OVERRIDE.value)
    print(
      f"  nbody: {mjm.nbody} nv: {mjm.nv} ngeom: {mjm.ngeom} nu: {mjm.nu}\n"
      f"  solver: {mujoco.mjtSolver(mjm.opt.solver).name} cone: {mujoco.mjtCone(mjm.opt.cone).name}"
      f" iterations: {mjm.opt.iterations} ls_iterations: {mjm.opt.ls_iterations}\n"
      f"  integrator: {mujoco.mjtIntegrator(mjm.opt.integrator).name}\n"
    )
    print(f"MuJoCo C simulating with dt = {mjm.opt.timestep:.3f}...")
  else:
    wp.config.quiet = flags.FLAGS["verbosity"].value < 1
    wp.init()
    if _CLEAR_KERNEL_CACHE.value:
      wp.clear_kernel_cache()

    with wp.ScopedDevice(_DEVICE.value):
      m = mjw.put_model(mjm)
      override_model(m, _OVERRIDE.value)
      broadphase, filter = mjw.BroadphaseType(m.opt.broadphase).name, mjw.BroadphaseFilter(m.opt.broadphase_filter).name
      solver, cone = mjw.SolverType(m.opt.solver).name, mjw.ConeType(m.opt.cone).name
      integrator = mjw.IntegratorType(m.opt.integrator).name
      iterations, ls_iterations = m.opt.iterations, m.opt.ls_iterations
      ls_str = f"{'parallel' if m.opt.ls_parallel else 'iterative'} linesearch iterations: {ls_iterations}"
      print(
        f"  nbody: {m.nbody} nv: {m.nv} ngeom: {m.ngeom} nu: {m.nu} is_sparse: {m.opt.is_sparse}\n"
        f"  broadphase: {broadphase} broadphase_filter: {filter}\n"
        f"  solver: {solver} cone: {cone} iterations: {iterations} {ls_str}\n"
        f"  integrator: {integrator} graph_conditional: {m.opt.graph_conditional}"
      )
      d = mjw.put_data(mjm, mjd, nconmax=_NCONMAX.value, njmax=_NJMAX.value)
      print(f"Data\n  nworld: {d.nworld} nconmax: {d.naconmax / d.nworld} njmax: {d.njmax}\n")
      graph = _compile_step(m, d)
      print(f"MuJoCo Warp simulating with dt = {m.opt.timestep.numpy()[0]:.3f}...")

  with mujoco.viewer.launch_passive(mjm, mjd, key_callback=key_callback) as viewer:
    opt = copy.copy(mjm.opt)

    while True:
      start = time.time()

      if ctrls is not None and ctrlid < len(ctrls):
        mjd.ctrl[:] = ctrls[ctrlid]
        ctrlid += 1

      if _ENGINE.value == EngineOptions.C:
        mujoco.mj_step(mjm, mjd)
      else:  # mjwarp
        wp.copy(d.ctrl, wp.array([mjd.ctrl.astype(np.float32)]))
        wp.copy(d.act, wp.array([mjd.act.astype(np.float32)]))
        wp.copy(d.xfrc_applied, wp.array([mjd.xfrc_applied.astype(np.float32)]))
        wp.copy(d.qpos, wp.array([mjd.qpos.astype(np.float32)]))
        wp.copy(d.qvel, wp.array([mjd.qvel.astype(np.float32)]))
        wp.copy(d.time, wp.array([mjd.time], dtype=wp.float32))

        # if the user changed an option in the MuJoCo Simulate UI, go ahead and recompile the step
        # TODO: update memory tied to option max iterations
        if mjm.opt != opt:
          opt = copy.copy(mjm.opt)
          m = mjw.put_model(mjm)
          graph = _compile_step(m, d)

        if _VIEWER_GLOBAL_STATE["running"]:
          wp.capture_launch(graph)
          wp.synchronize()
        elif _VIEWER_GLOBAL_STATE["step_once"]:
          _VIEWER_GLOBAL_STATE["step_once"] = False
          wp.capture_launch(graph)
          wp.synchronize()

        mjw.get_data_into(mjd, mjm, d)

      viewer.sync()

      elapsed = time.time() - start
      if elapsed < mjm.opt.timestep:
        time.sleep(mjm.opt.timestep - elapsed)


def main():
  # absl flags assumes __main__ is the main running module for printing usage documentation
  # pyproject bin scripts break this assumption, so manually set argv and docstring
  sys.argv[0] = "mujoco_warp.viewer"
  sys.modules["__main__"].__doc__ = __doc__
  app.run(_main)


if __name__ == "__main__":
  main()
