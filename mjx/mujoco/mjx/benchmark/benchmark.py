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
    ('humanoid', 'TPU v5 lite'): 1024,
    ('barkour', 'TPU v5 lite'): 1024,
    ('shadow_hand', 'TPU v5 lite'): 1024,
    ('humanoid', 'Tesla V100-SXM2-16GB'): 8192,
    ('barkour', 'Tesla V100-SXM2-16GB'): 8192,
    ('shadow_hand', 'Tesla V100-SXM2-16GB'): 4096,
    ('humanoid', 'cpu'): 64,
    ('barkour', 'cpu'): 64,
    ('shadow_hand', 'cpu'): 64,
}

flags.DEFINE_string('model', 'humanoid', 'Model to benchmark')
flags.DEFINE_string('device', 'cpu', 'Device benchmark is running on')


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
  m = mjx.device_put(m)

  def init(rng):
    d = mjx.make_data(m)
    qvel = 0.01 * jax.random.normal(rng, shape=(m.nv,))
    d = d.replace(qvel=qvel)
    return d

  def step(d):
    return mjx.step(m, d)

  batch_size = _BATCH_SIZE[(FLAGS.model, jax.devices()[0].device_kind)]
  _measure_fn(state, init, step, batch_size=batch_size)


if __name__ == '__main__':
  FLAGS(sys.argv)
  benchmark.register(_run, name=FLAGS.model + '_' + FLAGS.device)
  benchmark.main()
