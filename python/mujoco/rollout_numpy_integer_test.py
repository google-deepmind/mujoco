# Copyright 2026 DeepMind Technologies Limited
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
"""Regression tests for NumPy integer rollout sizing arguments."""

from absl.testing import absltest
import mujoco
from mujoco import rollout
import numpy as np


class RolloutNumpyIntegerTest(absltest.TestCase):

  def test_numpy_integer_nstep_and_chunk_size(self):
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body><freejoint/><geom size="0.1"/></body></worldbody></mujoco>'
    )
    data = mujoco.MjData(model)
    nstate = mujoco.mj_stateSize(
        model, mujoco.mjtState.mjSTATE_FULLPHYSICS
    )
    initial_state = np.zeros((1, nstate), dtype=mujoco.MJTNUM_DTYPE)

    state, _ = rollout.rollout(
        model, data, initial_state, nstep=np.int64(3)
    )
    self.assertEqual(state.shape[1], 3)

    state, _ = rollout.rollout(
        model,
        data,
        initial_state,
        nstep=np.int64(3),
        chunk_size=np.int64(1),
    )
    self.assertEqual(state.shape[1], 3)


if __name__ == '__main__':
  absltest.main()
