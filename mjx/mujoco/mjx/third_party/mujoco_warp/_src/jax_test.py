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

import os

import warp as wp
from absl.testing import absltest
from absl.testing import parameterized

import mujoco_warp as mjwarp
from mujoco.mjx.third_party.mujoco_warp._src.test_util import fixture

# TODO(team): JAX test is temporary, remove after we land MJX:Warp


class JAXTest(parameterized.TestCase):
  @parameterized.product(xml=("pendula.xml", "humanoid/humanoid.xml"), graph_conditional=(True, False))
  def test_jax(self, xml, graph_conditional):
    os.environ["XLA_FLAGS"] = "--xla_gpu_graph_min_graph_size=1"
    # Force JAX to allocate memory on demand and deallocate when not needed (slow)
    os.environ["XLA_PYTHON_CLIENT_ALLOCATOR"] = "platform"

    try:
      import jax
    except ImportError:
      self.skipTest("JAX not installed")

    from jax import numpy as jp
    from mujoco.mjx.third_party.warp.jax_experimental import ffi

    if jax.default_backend() != "gpu":
      self.skipTest("JAX default backend is not GPU")

    NWORLDS = 2
    NCONTACTS = 16
    UNROLL_LENGTH = 1

    mjm, _, m, d = fixture(
      xml,
      nworld=NWORLDS,
      nconmax=NWORLDS * NCONTACTS,
      njmax=NCONTACTS * 4,
      iterations=1,
      ls_iterations=4,
      kick=True,
    )
    m.opt.graph_conditional = graph_conditional

    def warp_step(
      qpos_in: wp.array(dtype=wp.float32, ndim=2),
      qvel_in: wp.array(dtype=wp.float32, ndim=2),
      qpos_out: wp.array(dtype=wp.float32, ndim=2),
      qvel_out: wp.array(dtype=wp.float32, ndim=2),
    ):
      wp.copy(d.qpos, qpos_in)
      wp.copy(d.qvel, qvel_in)
      mjwarp.step(m, d)
      wp.copy(qpos_out, d.qpos)
      wp.copy(qvel_out, d.qvel)

    def unroll(qpos, qvel):
      def step(carry, _):
        qpos, qvel = carry
        qpos, qvel = warp_step_fn(qpos, qvel)
        return (qpos, qvel), None

      (qpos, qvel), _ = jax.lax.scan(step, (qpos, qvel), length=UNROLL_LENGTH)

      return qpos, qvel

    warp_step_fn = ffi.jax_callable(
      warp_step,
      num_outputs=2,
      output_dims={"qpos_out": (NWORLDS, mjm.nq), "qvel_out": (NWORLDS, mjm.nv)},
      graph_mode=ffi.GraphMode.WARP,
    )

    # temp qpos0 array to get the right numpy shape
    qpos0_temp = wp.array(ptr=m.qpos0.ptr, shape=(1,) + m.qpos0.shape[1:], dtype=wp.float32)
    jax_qpos = jp.tile(jp.array(qpos0_temp), (NWORLDS, 1))
    jax_qvel = jp.zeros((NWORLDS, m.nv))

    jax_unroll_fn = jax.jit(unroll).lower(jax_qpos, jax_qvel).compile()
    jax_unroll_fn(jax_qpos, jax_qvel)


if __name__ == "__main__":
  wp.init()
  absltest.main()
