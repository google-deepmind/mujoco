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

"""Shared utilities and flags for MJWarp CLI tools."""

import time
from typing import Callable, Tuple, get_type_hints

import mujoco
import numpy as np
import warp as wp
from absl import app
from absl import flags
from etils import epath

import mujoco.mjx.third_party.mujoco_warp as mjw
from mujoco.mjx.third_party.mujoco_warp._src import warp_util
from mujoco.mjx.third_party.mujoco_warp._src.io import find_keys
from mujoco.mjx.third_party.mujoco_warp._src.io import make_trajectory
from mujoco.mjx.third_party.mujoco_warp._src.io import override_model
from mujoco.mjx.third_party.mujoco_warp._src.util_misc import halton

# shared flags for cli tool
NWORLD = flags.DEFINE_integer("nworld", 8192, "number of parallel rollouts")
NSTEP = flags.DEFINE_integer("nstep", 1000, "number of steps per rollout")
NCONMAX = flags.DEFINE_integer("nconmax", None, "override maximum number of contacts per world")
NJMAX = flags.DEFINE_integer("njmax", None, "override maximum number of constraints per world")
NJMAX_NNZ = flags.DEFINE_integer("njmax_nnz", None, "override maximum number of non-zeros in constraint Jacobian")
NCCDMAX = flags.DEFINE_integer("nccdmax", None, "override maximum number of CCD contacts per world")
OVERRIDE = flags.DEFINE_multi_string("override", [], "Model overrides (notation: foo.bar = baz)", short_name="o")
KEYFRAME = flags.DEFINE_integer("keyframe", 0, "keyframe to initialize simulation.")
EVENT_TRACE = flags.DEFINE_bool("event_trace", False, "print an event trace report")
NOISE_STD = flags.DEFINE_float("noise_std", 0.01, "add noise to ctrl signal (standard deviation)")
NOISE_RATE = flags.DEFINE_float("noise_rate", 0.1, "add noise to ctrl signal (noise rate)")

DEVICE = flags.DEFINE_string("device", None, "override the default Warp device")
REPLAY = flags.DEFINE_string("replay", None, "keyframe sequence to replay, keyframe name must prefix match")

RENDER_WIDTH = flags.DEFINE_integer("render_width", 64, "render width (pixels)")
RENDER_HEIGHT = flags.DEFINE_integer("render_height", 64, "render height (pixels)")
RENDER_RGB = flags.DEFINE_bool("render_rgb", True, "render RGB image")
RENDER_DEPTH = flags.DEFINE_bool("render_depth", True, "render depth image")
RENDER_TEXTURES = flags.DEFINE_bool("render_textures", True, "use textures")
RENDER_SHADOWS = flags.DEFINE_bool("render_shadows", False, "use shadows")


def load_model(path: epath.Path) -> mujoco.MjModel:
  """Load a MuJoCo model from a path, handling resources and plugins."""
  if not path.exists():
    resource_path = epath.resource_path("mjx") / "third_party/mujoco_warp" / path
    if not resource_path.exists():
      raise FileNotFoundError(f"file not found: {path}\nalso tried: {resource_path}")
    path = resource_path

  if path.suffix == ".mjb":
    return mujoco.MjModel.from_binary_path(path.as_posix())

  spec = mujoco.MjSpec.from_file(path.as_posix())
  if any(p.plugin_name.startswith("mujoco.sdf") for p in spec.plugins):
    from mujoco.mjx.third_party.mujoco_warp.test_data.collision_sdf.utils import register_sdf_plugins as register_sdf_plugins

    register_sdf_plugins(mjw)

  mjm = spec.compile()

  if OVERRIDE.value:
    override_model(mjm, OVERRIDE.value)

  return mjm


@wp.kernel
def _ctrl_noise(
  # Model:
  opt_timestep: wp.array[float],
  actuator_ctrllimited: wp.array[bool],
  actuator_ctrlrange: wp.array2d[wp.vec2],
  # Data in:
  ctrl_in: wp.array2d[float],
  # In:
  ctrl_center: wp.array[float],
  step: int,
  ctrlnoisestd: float,
  ctrlnoiserate: float,
  # Data out:
  ctrl_out: wp.array2d[float],
):
  worldid, actid = wp.tid()

  # convert rate and scale to discrete time (Ornstein-Uhlenbeck)
  rate = wp.exp(-opt_timestep[worldid % opt_timestep.shape[0]] / ctrlnoiserate)
  scale = ctrlnoisestd * wp.sqrt(1.0 - rate * rate)

  midpoint = 0.0
  halfrange = 1.0
  ctrlrange = actuator_ctrlrange[worldid % actuator_ctrlrange.shape[0], actid]
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


def init_structs(
  fn: Callable[..., None], mjm: mujoco.MjModel
) -> Tuple[mjw.Model, mjw.Data, mjw.RenderContext | None, list[np.ndarray] | None]:
  """Initialize device structs."""
  mjd = mujoco.MjData(mjm)
  ctrls = None
  if REPLAY.value:
    keys = find_keys(mjm, REPLAY.value)
    if not keys:
      raise app.UsageError(f"Key prefix not found: {REPLAY.value}")
    ctrls = make_trajectory(mjm, keys)
    mujoco.mj_resetDataKeyframe(mjm, mjd, keys[0])
  elif mjm.nkey > 0 and KEYFRAME.value > -1:
    mujoco.mj_resetDataKeyframe(mjm, mjd, KEYFRAME.value)
    ctrls = [mjd.ctrl.copy() for _ in range(NSTEP.value)]

  with wp.ScopedDevice(wp.get_device(DEVICE.value)):
    m = mjw.put_model(mjm)
    if OVERRIDE.value:
      override_model(m, OVERRIDE.value)
    d = mjw.put_data(
      mjm, mjd, nworld=NWORLD.value, nconmax=NCONMAX.value, njmax=NJMAX.value, njmax_nnz=NJMAX_NNZ.value, nccdmax=NCCDMAX.value
    )

    if mjw.RenderContext not in get_type_hints(fn).values():
      return m, d, None, ctrls

    rc = mjw.create_render_context(
      mjm,
      NWORLD.value,
      (RENDER_WIDTH.value, RENDER_HEIGHT.value),
      RENDER_RGB.value,
      RENDER_DEPTH.value,
      RENDER_TEXTURES.value,
      RENDER_SHADOWS.value,
    )

    return m, d, rc, ctrls


def unroll(
  fn: Callable[..., None],
  m: mjw.Model,
  d: mjw.Data,
  rc: mjw.RenderContext | None,
  callback: Callable[[int, dict, float], None] | None = None,
  ctrls: list[np.ndarray] | None = None,
) -> dict:
  """Unroll a function on batched Data and return some statistics.

  Args:
    fn: Function to unroll (e.g. mjw.step).
    m: Model.
    d: Data.
    rc: Render context (optional).
    callback: Optional callback called after each step with (step count, trace, latency).
    ctrls: Optional control trajectory.

  Returns:
    jit_duration: Time to JIT capture the function.
  """
  with wp.ScopedDevice(wp.get_device(DEVICE.value)):
    with warp_util.EventTracer(enabled=EVENT_TRACE.value) as tracer:
      jit_beg = time.perf_counter()
      with wp.ScopedCapture() as capture:
        fn(*(m, d) if rc is None else (m, d, rc))
      jit_end = time.perf_counter()

      for i in range(NSTEP.value):
        with wp.ScopedStream(wp.get_stream()):
          if ctrls is not None:
            center = wp.array(ctrls[i], dtype=wp.float32)
            wp.launch(
              _ctrl_noise,
              dim=(d.nworld, m.nu),
              inputs=[
                m.opt.timestep,
                m.actuator_ctrllimited,
                m.actuator_ctrlrange,
                d.ctrl,
                center,
                i,
                NOISE_STD.value,
                NOISE_RATE.value,
              ],
              outputs=[d.ctrl],
            )
            wp.synchronize()

          run_beg = time.perf_counter()
          wp.capture_launch(capture.graph)
          wp.synchronize()
          run_end = time.perf_counter()

        if callback:
          callback(i, tracer.trace(), run_end - run_beg)

  return jit_end - jit_beg
