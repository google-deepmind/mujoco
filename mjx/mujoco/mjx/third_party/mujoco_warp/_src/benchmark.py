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

"""Utilities for benchmarking MuJoCo Warp."""

import importlib
import os
import time
from typing import Callable, Optional, Tuple

import mujoco
import numpy as np
import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src import forward
from mujoco.mjx.third_party.mujoco_warp._src import io
from mujoco.mjx.third_party.mujoco_warp._src import warp_util
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import Model
from mujoco.mjx.third_party.mujoco_warp._src.util_misc import halton


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
  opt_timestep: wp.array(dtype=float),
  actuator_ctrllimited: wp.array(dtype=bool),
  actuator_ctrlrange: wp.array2d(dtype=wp.vec2),
  # Data in:
  ctrl_in: wp.array2d(dtype=float),
  # In:
  ctrl_center: wp.array1d(dtype=float),
  step: int,
  ctrlnoisestd: float,
  ctrlnoiserate: float,
  # Data out:
  ctrl_out: wp.array2d(dtype=float),
):
  worldid, actid = wp.tid()

  # convert rate and scale to discrete time (Ornstein-Uhlenbeck)
  rate = wp.exp(-opt_timestep[0] / ctrlnoiserate)
  scale = ctrlnoisestd * wp.sqrt(1.0 - rate * rate)

  midpoint = 0.0
  halfrange = 1.0
  ctrlrange = actuator_ctrlrange[0, actid]
  is_limited = actuator_ctrllimited[actid]
  if is_limited:
    midpoint = 0.5 * (ctrlrange[1] + ctrlrange[0])
    halfrange = 0.5 * (ctrlrange[1] - ctrlrange[0])
  if ctrl_center.shape[0] > 0:
    midpoint = ctrl_center[actid]

  # exponential convergence to midpoint at ctrlnoiserate
  ctrl = rate * ctrl_in[worldid, actid] + (1.0 - rate) * midpoint

  # add noise
  ctrl += scale * halfrange * (2.0 * halton((step + 1) * (worldid + 1), actid + 2) - 1.0)

  # clip to range if limited
  if is_limited:
    ctrl = wp.clamp(ctrl, ctrlrange[0], ctrlrange[1])

  ctrl_out[worldid, actid] = ctrl


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
    fn: Function to benchmark.
    m: The model containing kinematic and dynamic information (device).
    d: The data object containing the current state and output information (device).
    nstep: Number of timesteps.
    ctrls: Control sequence to apply during benchmarking.
    event_trace: If True, time routines decorated with @event_scope.
    measure_alloc: If True, record number of contacts and constraints.
    measure_solver_niter: If True, record the number of solver iterations.

  Returns:
    - Time to JIT fn.
    - Total time to run the benchmark.
    - Trace.
    - Number of contacts.
    - Number of constraints.
    - Number of solver iterations.
    - Number of converged worlds.
  """
  trace = {}
  nacon, nefc, solver_niter = [], [], []
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
          inputs=[m.opt.timestep, m.actuator_ctrllimited, m.actuator_ctrlrange, d.ctrl, center, i, 0.01, 0.1],
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
        nacon.append(np.max([d.nacon.numpy()[0], d.ncollision.numpy()[0]]))
        nefc.append(np.max(d.nefc.numpy()))
      if measure_solver_niter:
        solver_niter.append(d.solver_niter.numpy())

    nsuccess = np.sum(~np.any(np.isnan(d.qpos.numpy()), axis=1))
    run_duration = np.sum(time_vec)

  return jit_duration, run_duration, trace, nacon, nefc, solver_niter, nsuccess


class BenchmarkSuite:
  """Base suite for all model benchmarks."""

  path = ""
  batch_size = -1
  nconmax = -1
  njmax = -1
  nstep = 1000
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
      keys = io.find_keys(mjm, self.replay)
      if not keys:
        raise ValueError(f"Key prefix not find: {self.replay}")
      ctrls = io.make_trajectory(mjm, keys)
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

    jit_duration, _, trace, _, _, solver_niter, _ = benchmark(forward.step, m, d, self.nstep, ctrls, True, False, True)
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
