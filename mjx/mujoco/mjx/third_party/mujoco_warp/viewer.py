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

import ast
import enum
import logging
import pickle
import sys
import time
from typing import Sequence, Union

import mujoco
import mujoco.viewer
import numpy as np
import warp as wp
from absl import app
from absl import flags
from etils import epath

import mujoco_warp as mjw


class EngineOptions(enum.IntEnum):
  WARP = 0
  C = 1


_CLEAR_KERNEL_CACHE = flags.DEFINE_bool("clear_kernel_cache", False, "Clear kernel cache (to calculate full JIT time)")
_ENGINE = flags.DEFINE_enum_class("engine", EngineOptions.WARP, EngineOptions, "Simulation engine")
_NCONMAX = flags.DEFINE_integer("nconmax", None, "Maximum number of contacts.")
_NJMAX = flags.DEFINE_integer("njmax", None, "Maximum number of constraints per world.")
_OVERRIDE = flags.DEFINE_multi_string("override", [], "Model overrides (notation: foo.bar = baz)", short_name="o")
_DEVICE = flags.DEFINE_string("device", None, "override the default Warp device")


_VIEWER_GLOBAL_STATE = {"running": True, "step_once": False}


def key_callback(key: int) -> None:
  if key == 32:  # Space bar
    _VIEWER_GLOBAL_STATE["running"] = not _VIEWER_GLOBAL_STATE["running"]
    logging.info("RUNNING = %s", _VIEWER_GLOBAL_STATE["running"])
  elif key == 46:  # period
    _VIEWER_GLOBAL_STATE["step_once"] = True


def _load_model(path: epath.Path) -> mujoco.MjModel:
  if not path.exists():
    resource_path = epath.resource_path("mujoco_warp") / path
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


def _override(model: Union[mjw.Model, mujoco.MjModel]):
  enum_fields = {
    "opt.integrator": mjw.IntegratorType,
    "opt.cone": mjw.ConeType,
    "opt.solver": mjw.SolverType,
    "opt.broadphase": mjw.BroadphaseType,
    "opt.broadphase_filter": mjw.BroadphaseFilter,
  }
  for override in _OVERRIDE.value:
    if "=" not in override:
      raise app.UsageError(f"Invalid override format: {override}")
    key, val = override.split("=", 1)
    key, val = key.strip(), val.strip()

    if key in enum_fields:
      try:
        val = str(enum_fields[key][val.upper()])
      except KeyError:
        raise app.UsageError(f"Unrecognized enum value: {val}")

    obj, attrs = model, key.split(".")
    for i, attr in enumerate(attrs):
      if not hasattr(obj, attr):
        raise app.UsageError(f"Unrecognized model field: {key}")
      if i < len(attrs) - 1:
        obj = getattr(obj, attr)
      else:
        try:
          val = type(getattr(obj, attr))(ast.literal_eval(val))
        except (SyntaxError, ValueError):
          raise app.UsageError(f"Unrecognized value for field: {key}")

        setattr(obj, attr, val)


def _compile_step(m, d):
  mjw.step(m, d)
  # double warmup to work around issues with compilation during graph capture:
  mjw.step(m, d)
  # capture the whole step function as a CUDA graph
  with wp.ScopedCapture() as capture:
    mjw.step(m, d)
  return capture.graph


def _main(argv: Sequence[str]) -> None:
  """Runs viewer app."""
  if len(argv) < 2:
    raise app.UsageError("Missing required input: mjcf path.")
  elif len(argv) > 2:
    raise app.UsageError("Too many command-line arguments.")

  mjm = _load_model(epath.Path(argv[1]))
  mjd = mujoco.MjData(mjm)
  mujoco.mj_forward(mjm, mjd)

  if _ENGINE.value == EngineOptions.C:
    _override(mjm)
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
      _override(m)
      mjm_hash = pickle.dumps(mjm)
      broadphase, filter = mjw.BroadphaseType(m.opt.broadphase).name, mjw.BroadphaseFilter(m.opt.broadphase_filter).name
      solver, cone = mjw.SolverType(m.opt.solver).name, mjw.ConeType(m.opt.cone).name
      integrator = mjw.IntegratorType(m.opt.integrator).name
      iterations, ls_iterations, ls_parallel = m.opt.iterations, m.opt.ls_iterations, m.opt.ls_parallel
      print(
        f"  nbody: {m.nbody} nv: {m.nv} ngeom: {m.ngeom} nu: {m.nu} is_sparse: {m.opt.is_sparse}\n"
        f"  broadphase: {broadphase} broadphase_filter: {filter}\n"
        f"  solver: {solver} cone: {cone} iterations: {iterations} ls_iterations: {ls_iterations} ls_parallel: {ls_parallel}\n"
        f"  integrator: {integrator} graph_conditional: {m.opt.graph_conditional}"
      )
      d = mjw.put_data(mjm, mjd, nconmax=_NCONMAX.value, njmax=_NJMAX.value)
      print(f"Data\n  nworld: {d.nworld} nconmax: {d.nconmax} njmax: {d.njmax}\n")
      print("Compiling physics step...", end="")
      start = time.time()
      graph = _compile_step(m, d)
      elapsed = time.time() - start
      print(f"done ({elapsed:0.2}s).")
      print(f"MuJoCo Warp simulating with dt = {m.opt.timestep.numpy()[0]:.3f}...")

  with mujoco.viewer.launch_passive(mjm, mjd, key_callback=key_callback) as viewer:
    while True:
      start = time.time()

      if _ENGINE.value == EngineOptions.C:
        mujoco.mj_step(mjm, mjd)
      else:  # mjwarp
        wp.copy(d.ctrl, wp.array([mjd.ctrl.astype(np.float32)]))
        wp.copy(d.act, wp.array([mjd.act.astype(np.float32)]))
        wp.copy(d.xfrc_applied, wp.array([mjd.xfrc_applied.astype(np.float32)]))
        wp.copy(d.qpos, wp.array([mjd.qpos.astype(np.float32)]))
        wp.copy(d.qvel, wp.array([mjd.qvel.astype(np.float32)]))
        wp.copy(d.time, wp.array([mjd.time], dtype=wp.float32))

        hash = pickle.dumps(mjm)
        if hash != mjm_hash:
          mjm_hash = hash
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
