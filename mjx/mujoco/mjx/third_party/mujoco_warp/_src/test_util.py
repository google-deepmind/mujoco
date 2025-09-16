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

"""Utilities for testing."""

import importlib
import os
import time
from typing import Callable, Optional, Tuple

import mujoco
import numpy as np
import warp as wp
from etils import epath

from mujoco.mjx.third_party.mujoco_warp._src import forward
from mujoco.mjx.third_party.mujoco_warp._src import io
from mujoco.mjx.third_party.mujoco_warp._src import warp_util
from mujoco.mjx.third_party.mujoco_warp._src.types import BroadphaseType
from mujoco.mjx.third_party.mujoco_warp._src.types import ConeType
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import DisableBit
from mujoco.mjx.third_party.mujoco_warp._src.types import EnableBit
from mujoco.mjx.third_party.mujoco_warp._src.types import IntegratorType
from mujoco.mjx.third_party.mujoco_warp._src.types import Model
from mujoco.mjx.third_party.mujoco_warp._src.types import SolverType
from mujoco.mjx.third_party.mujoco_warp._src.util_misc import halton


def fixture(
  fname: Optional[str] = None,
  xml: Optional[str] = None,
  keyframe: int = -1,
  actuation: bool = True,
  contact: bool = True,
  constraint: bool = True,
  equality: bool = True,
  spring: bool = True,
  damper: bool = True,
  gravity: bool = True,
  clampctrl: bool = True,
  filterparent: bool = True,
  qpos0: bool = False,
  kick: bool = False,
  energy: bool = False,
  eulerdamp: Optional[bool] = None,
  cone: Optional[ConeType] = None,
  integrator: Optional[IntegratorType] = None,
  solver: Optional[SolverType] = None,
  iterations: Optional[int] = None,
  ls_iterations: Optional[int] = None,
  ls_parallel: Optional[bool] = None,
  sparse: Optional[bool] = None,
  broadphase: Optional[BroadphaseType] = None,
  disableflags: Optional[int] = None,
  enableflags: Optional[int] = None,
  applied: bool = False,
  nstep: int = 3,
  seed: int = 42,
  nworld: int = None,
  nconmax: int = None,
  njmax: int = None,
):
  np.random.seed(seed)
  if fname is not None:
    path = epath.resource_path("mjx") / "third_party/mujoco_warp" / "test_data" / fname
    mjm = mujoco.MjModel.from_xml_path(path.as_posix())
  elif xml is not None:
    mjm = mujoco.MjModel.from_xml_string(xml)
  else:
    raise ValueError("either fname or xml must be provided")

  if not actuation:
    mjm.opt.disableflags |= DisableBit.ACTUATION
  if not contact:
    mjm.opt.disableflags |= DisableBit.CONTACT
  if not constraint:
    mjm.opt.disableflags |= DisableBit.CONSTRAINT
  if not equality:
    mjm.opt.disableflags |= DisableBit.EQUALITY
  if not spring:
    mjm.opt.disableflags |= DisableBit.SPRING
  if not damper:
    mjm.opt.disableflags |= DisableBit.DAMPER
  if not gravity:
    mjm.opt.disableflags |= DisableBit.GRAVITY
  if not clampctrl:
    mjm.opt.disableflags |= DisableBit.CLAMPCTRL
  if not eulerdamp:
    mjm.opt.disableflags |= DisableBit.EULERDAMP
  if not filterparent:
    mjm.opt.disableflags |= DisableBit.FILTERPARENT

  if energy:
    mjm.opt.enableflags |= EnableBit.ENERGY

  if cone is not None:
    mjm.opt.cone = cone
  if integrator is not None:
    mjm.opt.integrator = integrator
  if disableflags is not None:
    mjm.opt.disableflags |= disableflags
  if enableflags is not None:
    mjm.opt.enableflags |= enableflags
  if solver is not None:
    mjm.opt.solver = solver
  if iterations is not None:
    mjm.opt.iterations = iterations
  if ls_iterations is not None:
    mjm.opt.ls_iterations = ls_iterations
  if sparse is not None:
    if sparse:
      mjm.opt.jacobian = mujoco.mjtJacobian.mjJAC_SPARSE
    else:
      mjm.opt.jacobian = mujoco.mjtJacobian.mjJAC_DENSE

  mjd = mujoco.MjData(mjm)
  if keyframe > -1:
    mujoco.mj_resetDataKeyframe(mjm, mjd, keyframe)
  elif qpos0:
    mjd.qpos[:] = mjm.qpos0
  else:
    # set random qpos, underlying code should gracefully handle un-normalized quats
    mjd.qpos[:] = np.random.random(mjm.nq)

  if kick:
    # give the system a little kick to ensure we have non-identity rotations
    mjd.qvel = np.random.uniform(-0.01, 0.01, mjm.nv)
    mjd.ctrl = np.random.uniform(-0.1, 0.1, size=mjm.nu)
  if applied:
    mjd.qfrc_applied = np.random.uniform(-0.1, 0.1, size=mjm.nv)
    mjd.xfrc_applied = np.random.uniform(-0.1, 0.1, size=mjd.xfrc_applied.shape)
  if kick or applied:
    mujoco.mj_step(mjm, mjd, nstep)  # let dynamics get state significantly non-zero

  if mjm.nmocap:
    mjd.mocap_pos = np.random.random(mjd.mocap_pos.shape)
    mocap_quat = np.random.random(mjd.mocap_quat.shape)
    mjd.mocap_quat = mocap_quat

  mujoco.mj_forward(mjm, mjd)
  mjd.qacc_warmstart = mjd.qacc
  m = io.put_model(mjm)
  if ls_parallel is not None:
    m.opt.ls_parallel = ls_parallel
  if broadphase is not None:
    m.opt.broadphase = broadphase

  d = io.put_data(mjm, mjd, nworld=nworld, nconmax=nconmax, njmax=njmax)
  return mjm, mjd, m, d


def find_keys(model: mujoco.MjModel, keyname_prefix: str) -> list[int]:
  """Finds keyframes that start with keyname_prefix."""
  keys = []

  for keyid in range(model.nkey):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, keyid)
    if name.startswith(keyname_prefix):
      keys.append(keyid)

  return keys


def make_trajectory(model: mujoco.MjModel, keys: list[int]) -> np.ndarray:
  """Make a ctrl trajectory with linear interpolation."""
  ctrls = []
  prev_ctrl_key = np.zeros(model.nu, dtype=np.float64)
  prev_time, time = 0.0, 0.0

  for key in keys:
    ctrl_key, ctrl_time = model.key_ctrl[key], model.key_time[key]
    if not ctrls and ctrl_time != 0.0:
      raise ValueError("first keyframe must have time 0.0")
    elif ctrls and ctrl_time <= prev_time:
      raise ValueError("keyframes must be in time order")

    while time < ctrl_time:
      frac = (time - prev_time) / (ctrl_time - prev_time)
      ctrls.append(prev_ctrl_key * (1 - frac) + ctrl_key * frac)
      time += model.opt.timestep

    ctrls.append(ctrl_key)
    time += model.opt.timestep
    prev_ctrl_key = ctrl_key
    prev_time = time

  return np.array(ctrls)


def _sum(stack1, stack2):
  ret = {}
  for k in stack1:
    times1, sub_stack1 = stack1[k]
    times2, sub_stack2 = stack2[k]
    times = [t1 + t2 for t1, t2 in zip(times1, times2)]
    ret[k] = (times, _sum(sub_stack1, sub_stack2))
  return ret


@wp.kernel
def ctrl_noise(
  # Model:
  actuator_ctrllimited: wp.array(dtype=bool),
  actuator_ctrlrange: wp.array2d(dtype=wp.vec2),
  # In:
  ctrl_center: wp.array1d(dtype=float),
  step: int,
  ctrlnoise: float,
  # Data out:
  ctrl_out: wp.array2d(dtype=float),
):
  worldid, actid = wp.tid()

  center = 0.0
  radius = 1.0
  ctrlrange = actuator_ctrlrange[0, actid]
  if ctrl_center.shape[0] > 0:
    center = ctrl_center[actid]
  elif actuator_ctrllimited[actid]:
    center = (ctrlrange[1] + ctrlrange[0]) / 2.0
    radius = (ctrlrange[1] - ctrlrange[0]) / 2.0
  radius *= ctrlnoise
  noise = 2.0 * halton((step + 1) * (worldid + 1), actid + 2) - 1.0
  ctrl_out[worldid, actid] = center + radius * noise


def benchmark(
  fn: Callable[[Model, Data], None],
  m: Model,
  d: Data,
  nstep: int,
  ctrls: Optional[np.ndarray] = None,
  event_trace: bool = False,
  measure_alloc: bool = False,
  measure_solver_niter: bool = False,
) -> Tuple[float, float, dict, list, list, list, int]:
  """Benchmark a function of Model and Data.

  Args:
    fn (Callable[[Model, Data], None]): Function to benchmark.
    m (Model): The model containing kinematic and dynamic information (device).
    d (Data): The data object containing the current state and output information (device).
    nstep (int): Number of timesteps.
    ctrls (list, optional): control sequence to apply during benchmarking.
                            Default is None.
    event_trace (bool, optional): If True, time routines decorated with @event_scope.
                                  Default is False.
    measure_alloc (bool, optional): If True, record number of contacts and constraints.
                                    Default is False.
    measure_solver_niter (bool, False): If True, record the number of solver iterations.
                                        Default is False.
  Returns:
    float: Time to JIT fn.
    float: Total time to run the benchmark.
    dict: Trace.
    list: Number of contacts.
    list: Number of constraints.
    list: Number of solver iterations.
    int: Number of converged worlds.
  """

  trace = {}
  ncon, nefc, solver_niter = [], [], []
  center = wp.array([], dtype=wp.float32)

  with warp_util.EventTracer(enabled=event_trace) as tracer:
    # capture the whole function as a CUDA graph
    jit_beg = time.perf_counter()
    with wp.ScopedCapture() as capture:
      fn(m, d)
    jit_end = time.perf_counter()
    jit_duration = jit_end - jit_beg

    graph = capture.graph

    time_vec = np.zeros(nstep)
    for i in range(nstep):
      with wp.ScopedStream(wp.get_stream()):
        if ctrls is not None:
          center = wp.array(ctrls[i], dtype=wp.float32)
        wp.launch(
          ctrl_noise,
          dim=(d.nworld, m.nu),
          inputs=[m.actuator_ctrllimited, m.actuator_ctrlrange, center, i, 0.01],
          outputs=[d.ctrl],
        )
        wp.synchronize()

        run_beg = time.perf_counter()
        wp.capture_launch(graph)
        wp.synchronize()
        run_end = time.perf_counter()

      time_vec[i] = run_end - run_beg
      if trace:
        trace = _sum(trace, tracer.trace())
      else:
        trace = tracer.trace()
      if measure_alloc:
        ncon.append(np.max([d.ncon.numpy()[0], d.ncollision.numpy()[0]]))
        nefc.append(np.max(d.nefc.numpy()))
      if measure_solver_niter:
        solver_niter.append(d.solver_niter.numpy())

    nsuccess = np.sum(~np.any(np.isnan(d.qpos.numpy()), axis=1))
    run_duration = np.sum(time_vec)

  return jit_duration, run_duration, trace, ncon, nefc, solver_niter, nsuccess


class BenchmarkSuite:
  """Base suite for all model benchmarks."""

  path = ""
  batch_size = -1
  nconmax = -1
  njmax = -1
  param_names = ("function",)
  params = (
    "jit_duration",
    "solver_niter_mean",
    "solver_niter_p95",
    "device_memory_allocated",
    "step",
    "step.forward",
    "step.forward.fwd_position",
    "step.forward.fwd_position.kinematics",
    "step.forward.fwd_position.com_pos",
    "step.forward.fwd_position.camlight",
    "step.forward.fwd_position.crb",
    "step.forward.fwd_position.tendon_armature",
    "step.forward.fwd_position.collision",
    "step.forward.fwd_position.make_constraint",
    "step.forward.fwd_position.transmission",
    "step.forward.sensor_pos",
    "step.forward.fwd_velocity",
    "step.forward.fwd_velocity.com_vel",
    "step.forward.fwd_velocity.passive",
    "step.forward.fwd_velocity.rne",
    "step.forward.fwd_velocity.tendon_bias",
    "step.forward.sensor_vel",
    "step.forward.fwd_actuation",
    "step.forward.fwd_acceleration",
    "step.forward.fwd_acceleration.xfrc_accumulate",
    "step.forward.sensor_acc",
    "step.forward.solve",
  )
  number = 1
  rounds = 1
  sample_time = 0
  repeat = 1
  replay = ""

  def setup_cache(self):
    module = importlib.import_module(self.__module__)
    path = os.path.join(os.path.realpath(os.path.dirname(module.__file__)), self.path)
    mjm = mujoco.MjModel.from_xml_path(path)
    mjd = mujoco.MjData(mjm)
    ctrls = None

    if self.replay:
      keys = find_keys(mjm, self.replay)
      if not keys:
        raise ValueError(f"Key prefix not find: {self.replay}")
      ctrls = make_trajectory(mjm, keys)
      mujoco.mj_resetDataKeyframe(mjm, mjd, keys[0])

    if mjm.nkey > 0:
      mujoco.mj_resetDataKeyframe(mjm, mjd, 0)

    # TODO(team): mj_forward call shouldn't be necessary, but it is
    mujoco.mj_forward(mjm, mjd)

    wp.init()
    if os.environ.get("ASV_CACHE_KERNELS", "false").lower() == "false":
      wp.clear_kernel_cache()

    free_before = wp.get_device().free_memory
    m = io.put_model(mjm)
    d = io.put_data(mjm, mjd, self.batch_size, self.nconmax, self.njmax)
    free_after = wp.get_device().free_memory

    jit_duration, _, trace, _, _, solver_niter, _ = benchmark(forward.step, m, d, 1000, ctrls, True, False, True)
    metrics = {
      "jit_duration": jit_duration,
      "solver_niter_mean": np.mean(solver_niter),
      "solver_niter_p95": np.quantile(solver_niter, 0.95),
      "device_memory_allocated": free_before - free_after,
    }

    def tree_flatten(d, parent_k=""):
      ret = {}
      steps = self.batch_size * 1000
      for k, v in d.items():
        k = parent_k + "." + k if parent_k else k
        ret = ret | {k: 1e6 * v[0][0] / steps} | tree_flatten(v[1], k)
      return ret

    metrics = metrics | tree_flatten(trace)

    return metrics

  def track_metric(self, metrics, fn):
    return metrics[fn]
