# Copyright 2026 keeeeenw
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Opt-in public API tests: execute only with --run-metal on an idle GPU."""

import mujoco
import numpy as np
import pytest
import torch

from mujoco_metal import load_model
from mujoco_metal import Simulation


def test_public_airborne_cpu_step_and_continuation():
  model = load_model()
  sim = Simulation(model, batch_size=2)
  qpos = np.tile(model.key_qpos[0], (2, 1))
  qpos[:, 2] += 1
  qvel = np.zeros((2, 20))
  sim.reset(qpos, qvel)
  ctrl = torch.full((2, 14), 0.01, device="mps", dtype=torch.float32)
  data = mujoco.MjData(model)
  data.qpos[:] = qpos[0]
  data.ctrl[:] = 0.01
  for _ in range(4):
    sim.step(ctrl)
    mujoco.mj_step(model, data)
  state = sim.state_dict()
  np.testing.assert_allclose(state["qpos"][0], data.qpos, atol=1e-3, rtol=0)
  np.testing.assert_allclose(state["qvel"][0], data.qvel, atol=0.05, rtol=0)
  sim.step(ctrl)
  expected = sim.state_dict()
  sim.reset(env_ids=[1])
  sim.load_state_dict(state)
  sim.step(ctrl)
  actual = sim.state_dict()
  np.testing.assert_allclose(
      actual["qpos"], expected["qpos"], atol=1e-6, rtol=0
  )
  np.testing.assert_allclose(
      actual["qvel"], expected["qvel"], atol=1e-5, rtol=0
  )
  np.testing.assert_array_equal(actual["time"], expected["time"])


def test_public_ground_contacts_and_overflow_leave_state_unchanged():
  ctrl = torch.zeros((1, 14), device="mps", dtype=torch.float32)
  sim = Simulation(batch_size=1)
  result = sim.step(ctrl)
  assert int(result.ncon[0]) > 0
  bounded = Simulation(batch_size=1, contact_capacity=1)
  before = bounded.state_dict()
  with pytest.raises(RuntimeError, match="Metal"):
    bounded.step(ctrl)
  after = bounded.state_dict()
  for field in ("qpos", "qvel", "time"):
    np.testing.assert_array_equal(before[field], after[field])
