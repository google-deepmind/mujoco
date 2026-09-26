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

"""CPU contract tests, not substitutes for GPU physics qualification."""

import copy
import json
from pathlib import Path
import subprocess
import sys
import types
from unittest import mock

from absl.testing import absltest
from absl.testing import parameterized
import mujoco
import numpy as np
import torch

from mujoco_metal import model as model_lib
from mujoco_metal import simulation


class CpuTensorFactory:
  """Exercise reset/restore validation using CPU storage, without physics."""

  @staticmethod
  def tensor(data, device):
    return torch.tensor(data)


def cpu_storage_simulation():
  sim = simulation.Simulation.__new__(simulation.Simulation)
  sim._model = model_lib.load_model()
  sim._fingerprint = model_lib.model_fingerprint(sim._model)
  sim.batch_size = 2
  sim._torch = CpuTensorFactory()
  sim._qpos = torch.zeros((2, 21), dtype=torch.float32)
  sim._qvel = torch.zeros((2, 20), dtype=torch.float32)
  sim._time = np.zeros(2)
  sim.reset()
  return sim


class ApiTest(parameterized.TestCase):

  def test_lazy_import_and_cpu_preflight(self):
    code = """
import json, sys
import mujoco_metal
assert 'torch' not in sys.modules
from mujoco_metal.__main__ import preflight
report = preflight()
assert 'torch' not in sys.modules
print(json.dumps(report))
"""
    result = subprocess.run(
        [sys.executable, "-c", code],
        check=True,
        capture_output=True,
        text=True,
        cwd="/tmp",
    )
    report = json.loads(result.stdout)
    self.assertFalse(report["gpu_executed"])
    self.assertEqual((report["nq"], report["nv"], report["nu"]), (21, 20, 14))
    self.assertTrue(Path(report["shader_path"]).is_file())

  def test_profile_loads_assets_and_cpu_physics(self):
    model = model_lib.load_model()
    model_lib.validate_model(copy.copy(model))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    for _ in range(4):
      mujoco.mj_step(model, data)
    self.assertTrue(np.isfinite(data.qacc).all())
    self.assertGreater(data.ncon, 0)
    allowed = {(0, 27), (0, 73)}
    for contact in data.contact:
      self.assertIn(tuple(sorted((contact.geom1, contact.geom2))), allowed)

  @parameterized.parameters(
      "mass", "gravity", "integrator", "geom", "joint", "actuator"
  )
  def test_unsupported_model_rejected_before_gpu(self, change):
    model = model_lib.load_model()
    if change == "mass":
      model.body_mass[2] *= 1.1
    elif change == "gravity":
      model.opt.gravity[2] = -1
    elif change == "integrator":
      model.opt.integrator = mujoco.mjtIntegrator.mjINT_RK4
    elif change == "geom":
      model.geom_contype[7] = 2
    elif change == "joint":
      model.jnt_pos[1, 0] = 0.1
    else:
      model.actuator_gear[0, 0] = 2
    with self.assertRaisesRegex(ValueError, "Unsupported model"):
      simulation.Simulation(model)

  def test_version_rejected(self):
    with mock.patch.object(mujoco, "__version__", "3.14.1"):
      with self.assertRaisesRegex(RuntimeError, "requalification"):
        model_lib.load_model()

  @parameterized.parameters(
      {"batch_size": 0},
      {"batch_size": True},
      {"contact_capacity": 36},
      {"constraint_capacity": 129},
      {"constraint_capacity": 3},
  )
  def test_capacities_rejected_before_gpu(self, **kwargs):
    with self.assertRaises(ValueError):
      simulation.Simulation(**kwargs)

  def test_reset_restore_and_snapshot_ownership(self):
    sim = cpu_storage_simulation()
    saved = sim.state_dict()
    changed = saved["qpos"][[1]].copy()
    changed[:, 0] += 0.3
    sim.reset(changed, np.ones((1, 20)), env_ids=[1])
    np.testing.assert_array_equal(sim.state_dict()["qpos"][0], saved["qpos"][0])
    self.assertNotEqual(sim.state_dict()["qpos"][1, 0], saved["qpos"][1, 0])
    sim.load_state_dict(saved)
    np.testing.assert_array_equal(sim.state_dict()["qpos"], saved["qpos"])
    saved["qpos"][:] = 99
    self.assertFalse(np.all(sim.state_dict()["qpos"] == 99))
    sim.reset(env_ids=np.array([], dtype=int))

  @parameterized.parameters(
      "nan", "quaternion", "shape", "duplicate", "negative", "float_ids"
  )
  def test_invalid_reset_is_transactional(self, kind):
    sim = cpu_storage_simulation()
    before = sim.state_dict()
    pos = before["qpos"].copy()
    ids = [0, 1]
    if kind == "nan":
      pos[1, 0] = np.nan
    elif kind == "quaternion":
      pos[1, 3:7] = 0
    elif kind == "shape":
      pos = pos[:1]
    elif kind == "duplicate":
      ids = [1, 1]
    elif kind == "negative":
      ids = [-1, 1]
    else:
      ids = [0.0, 1.0]
    with self.assertRaises(ValueError):
      sim.reset(pos, before["qvel"], env_ids=ids)
    np.testing.assert_array_equal(sim.state_dict()["qpos"], before["qpos"])

  @parameterized.parameters(
      "profile",
      "model_fingerprint",
      "batch_size",
      "format_version",
      "time",
      "qvel",
  )
  def test_invalid_restore_is_transactional(self, key):
    sim = cpu_storage_simulation()
    before = sim.state_dict()
    broken = copy.deepcopy(before)
    broken[key] = np.array([np.nan, 0.0]) if key == "time" else "invalid"
    with self.assertRaises((ValueError, TypeError)):
      sim.load_state_dict(broken)
    np.testing.assert_array_equal(sim.state_dict()["qpos"], before["qpos"])
    np.testing.assert_array_equal(sim.state_dict()["time"], before["time"])

  @parameterized.parameters(
      "contact_overflow",
      "assembly_overflow",
      "cholesky_status",
      "solver_status",
      "integration_status",
      "qpos",
  )
  def test_status_and_finiteness_fail_closed(self, field):
    output = types.SimpleNamespace(
        **{
            key: torch.zeros(2, dtype=torch.int32)
            for key in (
                "contact_overflow",
                "assembly_overflow",
                "cholesky_status",
                "solver_status",
                "integration_status",
            )
        }
    )
    output.qpos = torch.zeros((2, 21))
    output.qvel = torch.zeros((2, 20))
    simulation.check_result(output, torch)
    if field == "qpos":
      output.qpos[1, 0] = float("nan")
    else:
      getattr(output, field)[1] = -1
    with self.assertRaises(RuntimeError):
      simulation.check_result(output, torch)

  def test_bounded_unconverged_status_is_explicit(self):
    output = types.SimpleNamespace(
        **{
            key: torch.zeros(2, dtype=torch.int32)
            for key in (
                "contact_overflow",
                "assembly_overflow",
                "cholesky_status",
                "solver_status",
                "integration_status",
            )
        }
    )
    output.qpos = torch.zeros((2, 21))
    output.qvel = torch.zeros((2, 20))
    output.solver_status[1] = 1
    simulation.check_result(output, torch)
    with self.assertRaisesRegex(RuntimeError, "solver"):
      simulation.check_result(output, torch, require_convergence=True)


if __name__ == "__main__":
  absltest.main()
