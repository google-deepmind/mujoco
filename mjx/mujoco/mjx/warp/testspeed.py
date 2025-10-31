# Copyright 2025 DeepMind Technologies Limited
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
"""Run benchmarks."""

import functools
import os
import time
from typing import Any, Callable, Sequence, Tuple

from absl import app
from absl import flags
import jax
import mujoco
from mujoco import mjx
from mujoco.mjx._src import test_util
from mujoco.mjx.warp import collision_driver as wp_collision
from mujoco.mjx.warp import forward as wp_forward
from mujoco.mjx.warp import smooth as wp_smooth
import mujoco.mjx.third_party.mujoco_warp as mjwarp
import warp as wp
from mujoco.mjx.third_party.warp.jax_experimental import ffi as warp_ffi

_MODELFILE = flags.DEFINE_string(
    'modelfile',
    'humanoid/humanoid.xml',
    'path to model',
)
_FUNCTION = flags.DEFINE_string(
    'function', 'kinematics', 'function to benchmark'
)
_NSTEP = flags.DEFINE_integer('nstep', 1000, 'number of steps per rollout')
_NENV = flags.DEFINE_integer('nenv', 8192, 'number of environments to simulate')
_UNROLL = flags.DEFINE_integer('unroll', 4, 'number of steps to unroll')
_NACONMAX = flags.DEFINE_integer('naconmax', 30_000, 'max contacts')
_NJMAX = flags.DEFINE_integer('njmax', 10, 'max constraints')
_WP_KERNEL_CACHE_DIR = flags.DEFINE_string(
    'wp_kernel_cache_dir',
    None,
    'Path to the Warp kernel cache directory.',
)
_COMPILER_OPTIONS = {'xla_gpu_graph_min_graph_size': 1}
jax_jit = functools.partial(jax.jit, compiler_options=_COMPILER_OPTIONS)


def _measure(fn, *args) -> Tuple[float, float]:
  """Reports jit time and op time for a function."""

  beg = time.perf_counter()
  compiled_fn = fn.lower(*args).compile()
  end = time.perf_counter()
  jit_time = end - beg

  # warmup
  result = compiled_fn(*args)
  jax.block_until_ready(result)

  times = []

  for i in range(1):
    beg = time.perf_counter()
    result = compiled_fn(*args)
    jax.block_until_ready(result)
    end = time.perf_counter()
    run_time = end - beg
    times.append(run_time)
    print('Measure run: ', i, f', run time: {run_time:.3f}')

  return jit_time, sum(times) / len(times)


def benchmark(
    m: mujoco.MjModel,
    mx: mjx.Model,
    step_fn: Callable[..., Any],
    nstep: int = 1000,
    nenv: int = 8192,
    unroll_steps: int = 4,
) -> Tuple[float, float, int]:
  """Benchmark a model."""

  @jax.vmap
  def init(key):
    d = mjx.make_data(
        m, impl=mx.impl, nconmax=_NCONMAX.value, njmax=_NJMAX.value
    )
    return d

  key = jax.random.split(jax.random.key(0), nenv)
  d = jax_jit(init)(key)
  jax.block_until_ready(d)

  @jax_jit
  def unroll(d):
    def fn(d, _):
      d = d.replace(qpos=d.qpos + 0 * d.qpos)
      return step_fn(mx, d), None

    return jax.lax.scan(fn, d, None, length=nstep, unroll=unroll_steps)

  jit_time, run_time = _measure(unroll, d)
  steps = nstep * nenv

  return jit_time, run_time, steps


def benchmark_raw_jax_warp(
    m: mujoco.MjModel,
    nstep: int = 1000,
    nenv: int = 8192,
    unroll_steps: int = 4,
    function: str = 'kinematics',
):
  if function not in ('kinematics', 'forward', 'step', 'collision'):
    raise NotImplementedError(
        f'{function} is not implemented for raw warp speed test.'
    )

  def warp_fn(
      qpos_in: wp.array2d(dtype=wp.float32),
      qpos_out: wp.array2d(dtype=wp.float32),
      time_: wp.array(dtype=wp.float32),
      xpos: wp.array2d(dtype=wp.vec3),
      xquat: wp.array2d(dtype=wp.quat),
      xmat: wp.array2d(dtype=wp.mat33),
      xipos: wp.array2d(dtype=wp.vec3),
      ximat: wp.array2d(dtype=wp.mat33),
      xanchor: wp.array2d(dtype=wp.vec3),
      xaxis: wp.array2d(dtype=wp.vec3),
      geom_xpos: wp.array2d(dtype=wp.vec3),
      geom_xmat: wp.array2d(dtype=wp.mat33),
  ):
    wp.copy(d.qpos, qpos_in)
    if function == 'kinematics':
      mjwarp.kinematics(mw, d)
    elif function == 'forward':
      mjwarp.forward(mw, d)
    elif function == 'step':
      mjwarp.step(mw, d)
    elif function == 'collision':
      mjwarp.collision(mw, d)
    else:
      raise NotImplementedError(f'{function} not implemented in speed test.')
    wp.copy(qpos_out, d.qpos)
    wp.copy(time_, d.time)
    wp.copy(xpos, d.xpos)
    wp.copy(xquat, d.xquat)
    wp.copy(xmat, d.xmat)
    wp.copy(xipos, d.xipos)
    wp.copy(ximat, d.ximat)
    wp.copy(xanchor, d.xanchor)
    wp.copy(xaxis, d.xaxis)
    wp.copy(geom_xpos, d.geom_xpos)
    wp.copy(geom_xmat, d.geom_xmat)

  def unroll(
      qpos,
      qpos_out,
      time_,
      xpos,
      xquat,
      xmat,
      xipos,
      ximat,
      xanchor,
      xaxis,
      geom_xpos,
      geom_xmat,
  ):
    def step(carry, _):
      qpos_, *_ = carry
      out = warp_fn_jax(qpos_ + 0.0 * qpos_)
      out = tuple(out)
      return (out[0],) + out, None

    (
        qpos,
        qpos_out,
        time_,
        xpos,
        xquat,
        xmat,
        xipos,
        ximat,
        xanchor,
        xaxis,
        geom_xpos,
        geom_xmat,
    ), _ = jax.lax.scan(
        step,
        (
            qpos,
            qpos_out,
            time_,
            xpos,
            xquat,
            xmat,
            xipos,
            ximat,
            xanchor,
            xaxis,
            geom_xpos,
            geom_xmat,
        ),
        length=nstep,
        unroll=unroll_steps,
    )

    return (
        qpos,
        qpos_out,
        time_,
        xpos,
        xquat,
        xmat,
        xipos,
        ximat,
        xanchor,
        xaxis,
        geom_xpos,
        geom_xmat,
    )

  output_dims = {
      'time_': (nenv,),
      'qpos_out': (nenv, m.nq),
      'xpos': (nenv, m.nbody, 3),
      'xquat': (nenv, m.nbody, 4),
      'xmat': (nenv, m.nbody, 3, 3),
      'xipos': (nenv, m.nbody, 3),
      'ximat': (nenv, m.nbody, 3, 3),
      'xanchor': (nenv, m.njnt, 3),
      'xaxis': (nenv, m.njnt, 3),
      'geom_xpos': (nenv, m.ngeom, 3),
      'geom_xmat': (nenv, m.ngeom, 3, 3),
  }
  warp_fn_jax = warp_ffi.jax_callable(
      warp_fn,
      num_outputs=11,
      output_dims=output_dims,
      graph_mode=warp_ffi.GraphMode.JAX,
  )

  @jax.vmap
  def init(key):
    d = mjx.make_data(m, impl='jax')
    return d

  key = jax.random.split(jax.random.key(0), nenv)
  dx = jax_jit(init)(key)
  d_ = mujoco.MjData(m)
  mw = mjwarp.put_model(m)
  d = mjwarp.put_data(
      m, d_, nworld=nenv, nconmax=_NCONMAX.value, njmax=_NJMAX.value
  )

  jax_unroll_fn = jax_jit(unroll)
  jit_time, run_time = _measure(
      jax_unroll_fn,
      dx.qpos,
      dx.qpos,
      dx.time,
      dx.xpos,
      dx.xquat,
      dx.xmat,
      dx.xipos,
      dx.ximat,
      dx.xanchor,
      dx.xaxis,
      dx.geom_xpos,
      dx.geom_xmat,
  )
  steps = nstep * nenv

  return jit_time, run_time, steps


def _compile_fn(fn, m, d):
  fn(m, d)
  fn(m, d)
  with wp.ScopedCapture() as capture:
    fn(m, d)
  return capture.graph


def benchmark_raw_warp(
    m: mujoco.MjModel,
    nstep: int = 1000,
    nenv: int = 8192,
    unroll_steps: int = 4,
    function: str = 'kinematics',
):
  """Benchmarks raw warp."""
  del unroll_steps
  if function not in ('kinematics', 'forward', 'step', 'collision'):
    raise NotImplementedError(
        f'{function} is not implemented for raw warp speed test.'
    )

  mw = mjwarp.put_model(m)
  dw = mjwarp.make_data(
      m, nworld=nenv, nconmax=_NCONMAX.value, njmax=_NJMAX.value
  )

  if function == 'kinematics':
    fn = mjwarp.kinematics
  elif function == 'forward':
    fn = mjwarp.forward
  elif function == 'step':
    fn = mjwarp.step
  elif function == 'collision':
    fn = mjwarp.collision
  else:
    raise NotImplementedError(f'{function} not implemented in speed test.')

  start = time.time()
  graph = _compile_fn(fn, mw, dw)
  jit_time = time.time() - start

  start = time.time()
  for _ in range(nstep):
    wp.capture_launch(graph)
    wp.synchronize()

  run_time = time.time() - start
  return jit_time, run_time, nstep * nenv


def _main(_: Sequence[str]):
  """Runs testpeed function."""
  os.environ['MJX_WARP_ENABLED'] = 'true'

  if _WP_KERNEL_CACHE_DIR.value:
    wp.config.kernel_cache_dir = _WP_KERNEL_CACHE_DIR.value

  modelfile = _MODELFILE.value
  function_ = _FUNCTION.value
  nstep, nenv, unroll = _NSTEP.value, _NENV.value, _UNROLL.value

  try:
    m = test_util.load_test_file(modelfile)
  except Exception as _:
    m = mujoco.MjModel.from_xml_path(modelfile)

  mx = mjx.put_model(m, impl='jax')
  mw = mjx.put_model(m, impl='warp')

  if function_ == 'kinematics':
    func_warp = jax.vmap(wp_smooth.kinematics, in_axes=(None, 0))
    func_jax = jax.vmap(mjx.kinematics, in_axes=(None, 0))
  elif function_ == 'forward':
    func_warp = jax.vmap(wp_forward.forward, in_axes=(None, 0))
    func_jax = jax.vmap(mjx.forward, in_axes=(None, 0))
  elif function_ == 'step':
    func_warp = jax.vmap(wp_forward.step, in_axes=(None, 0))
    func_jax = jax.vmap(mjx.step, in_axes=(None, 0))
  elif function_ == 'collision':
    func_warp = jax.vmap(wp_collision.collision, in_axes=(None, 0))
    func_jax = jax.vmap(mjx.collision, in_axes=(None, 0))
  else:
    raise ValueError(f'Unknown function: {function_}')

  print('testspeed.py:\n')
  print(f' modelfile            : {modelfile}')
  print(f' function             : {function_}')
  print(f' nenv                 : {nenv}')
  print(f' nstep                : {nstep}')
  print(f' timestep             : {m.opt.timestep}')
  print(f' unroll               : {unroll}\n')

  for name, mx_, op in (
      ('JAX WARP FFI', mw, func_warp),
      ('Pure JAX', mx, func_jax),
  ):
    if op is not None:
      jit_time, run_time, steps = benchmark(m, mx_, op, nstep, nenv, unroll)

      print(f' {name}:')
      print(f' JIT time             : {jit_time:.2f} s')
      print(f' simulation time      : {run_time:.2f} s')
      print(f' steps per second     : {steps / run_time:,.0f}')
      print(
          f' realtime factor      : {steps * m.opt.timestep / run_time:.2f} x'
      )
      print(f' time per step        : {1e6 * run_time / steps:.2f} µs\n')

  jit_time, run_time, steps = benchmark_raw_jax_warp(
      m, nstep, nenv, unroll, function=function_
  )
  print(' Pure JAX-WARP:')
  print(f' JIT time             : {jit_time:.2f} s')
  print(f' simulation time      : {run_time:.2f} s')
  print(f' steps per second     : {steps / run_time:,.0f}')
  print(f' realtime factor      : {steps * m.opt.timestep / run_time:.2f} x')
  print(f' time per step        : {1e6 * run_time / steps:.2f} µs\n')

  jit_time, run_time, steps = benchmark_raw_warp(
      m, nstep, nenv, unroll, function=function_
  )
  print(' Pure WARP:')
  print(f' JIT time             : {jit_time:.2f} s')
  print(f' simulation time      : {run_time:.2f} s')
  print(f' steps per second     : {steps / run_time:,.0f}')
  print(f' realtime factor      : {steps * m.opt.timestep / run_time:.2f} x')
  print(f' time per step        : {1e6 * run_time / steps:.2f} µs\n')


def main():
  app.run(_main)


if __name__ == '__main__':
  main()
