# Copyright 2023 DeepMind Technologies Limited
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
"""Run benchmarks on various devices."""

import time
from typing import Sequence, Tuple

from absl import app
from absl import flags
from etils import epath
import jax
import mujoco
from mujoco import mjx

FLAGS = flags.FLAGS

flags.DEFINE_string('mjcf', None, 'path to model', required=True)
flags.DEFINE_integer('nstep', 1000, 'number of steps per rollout')
flags.DEFINE_integer('batch_size', 1024, 'number of parallel rollouts')
flags.DEFINE_integer('unroll', 1, 'loop unroll length')
flags.DEFINE_enum('solver', 'cg', ['cg', 'newton'], 'constraint solver')
flags.DEFINE_integer('iterations', 1, 'number of solver iterations')
flags.DEFINE_integer('ls_iterations', 4, 'number of linesearch iterations')
flags.DEFINE_enum('output', 'text', ['text', 'tsv'], 'format to print results')


def _measure(fn, *args) -> Tuple[float, float]:
  """Reports jit time and op time for a function."""

  beg = time.perf_counter()
  compiled_fn = fn.lower(*args).compile()
  end = time.perf_counter()
  jit_time = end - beg

  beg = time.perf_counter()
  result = compiled_fn(*args)
  jax.block_until_ready(result)
  end = time.perf_counter()
  run_time = end - beg

  return jit_time, run_time


def _main(argv: Sequence[str]):
  """Benchmark a model."""

  f = epath.resource_path('mujoco.mjx') / 'test_data' / FLAGS.mjcf
  m = mujoco.MjModel.from_xml_path(f.as_posix())
  m.opt.solver = {
      'cg': mujoco.mjtSolver.mjSOL_CG,
      'newton': mujoco.mjtSolver.mjSOL_NEWTON,
  }[FLAGS.solver.lower()]
  m.opt.iterations = FLAGS.iterations
  m.opt.ls_iterations = FLAGS.ls_iterations
  m = mjx.put_model(m)

  if FLAGS.output == 'text':
    print(f"Rolling out {FLAGS.nstep} steps at dt = {m.opt.timestep:.3f}...")

  @jax.pmap
  def init(key):
    key = jax.random.split(key, FLAGS.batch_size // jax.device_count())

    @jax.vmap
    def random_init(key):
      d = mjx.make_data(m)
      qvel = 0.01 * jax.random.normal(key, shape=(m.nv,))
      d = d.replace(qvel=qvel)
      return d

    return random_init(key)

  key = jax.random.split(jax.random.key(0), jax.device_count())
  d = init(key)
  jax.block_until_ready(d)

  @jax.pmap
  def unroll(d):

    @jax.vmap
    def step(d, _):
      d = mjx.step(m, d)
      return d, None

    d, _ = jax.lax.scan(step, d, None, length=FLAGS.nstep, unroll=FLAGS.unroll)

    return d

  jit_time, run_time = _measure(unroll, d)
  steps = FLAGS.nstep * FLAGS.batch_size

  if FLAGS.output == 'text':
    print(f"""
Summary for {FLAGS.batch_size} parallel rollouts

 Total JIT time: {jit_time:.2f} s
 Total simulation time: {run_time:.2f} s
 Total steps per second: { steps / run_time:.0f}
 Total realtime factor: { steps * m.opt.timestep / run_time:.2f} x
 Total time per step: { 1e6 * run_time / steps:.2f} µs""")
  elif FLAGS.output == 'tsv':
    name = argv[0].split('/')[-1].replace('testspeed_', '')
    print(f"{name}\tjit: {jit_time:.2f}s\tsteps/second: {steps / run_time:.0f}")


def main():
  app.run(_main)


if __name__ == '__main__':
  main()
