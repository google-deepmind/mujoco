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

_PATHS = {
    'humanoid': 'benchmark/model/humanoid/humanoid.xml',
    'barkour': 'benchmark/model/barkour_v0/assets/barkour_v0_mjx.xml',
    'shadow_hand': 'benchmark/model/shadow_hand/scene_right.xml',
}

_BATCH_SIZE = {
    ('barkour', 'tpu_v5e'): 1024,
    ('humanoid', 'tpu_v5e'): 1024,
    ('shadow_hand', 'tpu_v5e'): 1024,
    ('barkour', 'gpu_a100'): 8192,
    ('humanoid', 'gpu_a100'): 8192,
    ('shadow_hand', 'gpu_a100'): 4096,
    ('barkour', 'cpu'): 64,
    ('humanoid', 'cpu'): 64,
    ('shadow_hand', 'cpu'): 64,
}

_SOLVER_CONFIG = {
    ('barkour', 'tpu_v5e'): (mujoco.mjtSolver.mjSOL_CG, 4, 6),
    ('humanoid', 'tpu_v5e'): (mujoco.mjtSolver.mjSOL_CG, 6, 6),
    ('shadow_hand', 'tpu_v5e'): (mujoco.mjtSolver.mjSOL_CG, 8, 6),
    ('humanoid', 'gpu_a100'): (mujoco.mjtSolver.mjSOL_NEWTON, 1, 4),
    ('barkour', 'gpu_a100'): (mujoco.mjtSolver.mjSOL_NEWTON, 1, 4),
    ('shadow_hand', 'gpu_a100'): (mujoco.mjtSolver.mjSOL_NEWTON, 1, 4),
    ('barkour', 'cpu'): (mujoco.mjtSolver.mjSOL_NEWTON, 1, 4),
    ('humanoid', 'cpu'): (mujoco.mjtSolver.mjSOL_NEWTON, 1, 4),
    ('shadow_hand', 'cpu'): (mujoco.mjtSolver.mjSOL_NEWTON, 1, 4),
}


flags.DEFINE_string('model', 'humanoid', 'Model to benchmark')
flags.DEFINE_enum('device', 'cpu', ('cpu', 'tpu_v5e', 'gpu_a100'),
                  'Device benchmark is running on')


def _measure_fn(state, init_fn, step_fn, batch_size: int = 1024) -> float:
  """Reports jit time and op time for a function."""

  step_count = 100 if FLAGS.device == 'cpu' else 1000

  @jax.jit
  def run_batch(seed: jp.ndarray):
    rngs = jax.random.split(jax.random.PRNGKey(seed), batch_size)
    init_state = jax.vmap(init_fn)(rngs)

    @jax.vmap
    def run(state):
      def step(state, _):
        state = step_fn(state)
        return state, ()

      return jax.lax.scan(step, state, (), length=step_count)

    return run(init_state)

  # run once to jit
  beg = time.perf_counter()
  jax.tree_util.tree_map(lambda x: x.block_until_ready(), run_batch(0))
  first_t = time.perf_counter() - beg

  times = []
  while state:
    beg = time.perf_counter()
    batch = run_batch(jp.array(len(times)))
    jax.tree_util.tree_map(lambda x: x.block_until_ready(), batch)
    times.append(time.perf_counter() - beg)

  op_time = jp.mean(jp.array(times))
  batch_sps = batch_size * step_count / op_time

  state.counters['jit_time'] = first_t - op_time
  state.counters['batch_sps'] = batch_sps


@benchmark.option.unit(benchmark.kSecond)
def _run(state: benchmark.State):
  """Benchmark a model."""

  f = epath.resource_path('mujoco.mjx') / _PATHS[FLAGS.model]
  m = mujoco.MjModel.from_xml_path(f.as_posix())
  m.opt.solver, m.opt.iterations, m.opt.ls_iterations = _SOLVER_CONFIG[
      (FLAGS.model, FLAGS.device)
  ]
  m = mjx.device_put(m)

  def init(rng):
    d = mjx.make_data(m)
    qvel = 0.01 * jax.random.normal(rng, shape=(m.nv,))
    d = d.replace(qvel=qvel)
    return d

  def step(d):
    return mjx.step(m, d)

  batch_size = _BATCH_SIZE[(FLAGS.model, FLAGS.device)]
  _measure_fn(state, init, step, batch_size=batch_size)


if __name__ == '__main__':
  FLAGS(sys.argv)
  benchmark.register(_run, name=FLAGS.model + '_' + FLAGS.device)
  benchmark.main()
