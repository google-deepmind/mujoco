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

import sys
import time

from absl import flags
from etils import epath
import google_benchmark as benchmark
import jax
from jax import numpy as jp
import mujoco
from mujoco import mjx

FLAGS = flags.FLAGS

flags.DEFINE_string('mjcf', None, 'path to model', required=True)
flags.DEFINE_integer('step_count', 1000, 'number of steps per rollout')
flags.DEFINE_integer('batch_size', 1024, 'number of parallel rollouts')
flags.DEFINE_integer('unroll', 1, 'loop unroll length')
flags.DEFINE_enum('solver', 'cg', ['cg', 'newton'], 'constraint solver')
flags.DEFINE_integer('iterations', 1, 'number of solver iterations')
flags.DEFINE_integer('ls_iterations', 4, 'number of linesearch iterations')


def _measure(state, init_fn, step_fn) -> float:
  """Reports jit time and op time for a function."""

  @jax.pmap
  def run_batch(seed: jp.ndarray):
    batch_size = FLAGS.batch_size // jax.device_count()
    rngs = jax.random.split(jax.random.PRNGKey(seed), batch_size)
    state = jax.vmap(init_fn)(rngs)

    @jax.vmap
    def step(state, _):
      state = step_fn(state)
      return state, None

    state, _ = jax.lax.scan(
        step, state, None, length=FLAGS.step_count, unroll=FLAGS.unroll
    )
    return state

  # run once to jit
  beg = time.perf_counter()
  seed = 0
  seeds = jp.arange(seed, seed + jax.device_count(), dtype=int)
  jax.tree_util.tree_map(lambda x: x.block_until_ready(), run_batch(seeds))
  first_t = time.perf_counter() - beg

  times = []
  while state:
    seed += jax.device_count()
    seeds = jp.arange(seed, seed + jax.device_count(), dtype=int)
    beg = time.perf_counter()
    jax.tree_util.tree_map(lambda x: x.block_until_ready(), run_batch(seeds))
    times.append(time.perf_counter() - beg)

  op_time = jp.mean(jp.array(times))
  batch_sps = FLAGS.batch_size * FLAGS.step_count / op_time

  state.counters['jit_time'] = first_t - op_time
  state.counters['batch_sps'] = batch_sps


@benchmark.option.unit(benchmark.kSecond)
def _run(state: benchmark.State):
  """Benchmark a model."""

  f = epath.resource_path('mujoco.mjx') / 'benchmark/model' / FLAGS.mjcf
  m = mujoco.MjModel.from_xml_path(f.as_posix())
  m.opt.solver = {
      'cg': mujoco.mjtSolver.mjSOL_CG,
      'newton': mujoco.mjtSolver.mjSOL_NEWTON,
  }[FLAGS.solver.lower()]
  m.opt.iterations = FLAGS.iterations
  m.opt.ls_iterations = FLAGS.ls_iterations
  m = mjx.device_put(m)

  def init(rng):
    d = mjx.make_data(m)
    qvel = 0.01 * jax.random.normal(rng, shape=(m.nv,))
    d = d.replace(qvel=qvel)
    return d

  def step(d):
    return mjx.step(m, d)

  _measure(state, init, step)


if __name__ == '__main__':
  FLAGS(sys.argv)
  benchmark.register(_run, name=sys.argv[0].split('/')[-1])
  benchmark.main()
