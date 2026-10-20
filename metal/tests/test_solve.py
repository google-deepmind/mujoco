# Copyright 2026 The MuJoCo Metal contributors
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

"""CPU metadata and opt-in MPS qualification for the dense SPD solve."""

import os
from types import SimpleNamespace

import numpy as np
import pytest

from mujoco_metal.smooth_solve import MetalDenseSolve

pytestmark = pytest.mark.gpu


def test_constructor_rejects_invalid_capacities_before_device_setup():
  with pytest.raises(ValueError, match="nv"):
    MetalDenseSolve(-1, 1)
  with pytest.raises(ValueError, match="batch_size"):
    MetalDenseSolve(1, 0)
  with pytest.raises(ValueError, match="nrhs"):
    MetalDenseSolve(1, 1, 0)
  with pytest.raises(TypeError, match="nv"):
    MetalDenseSolve(1.5, 1)


def test_metadata_rejection_does_not_touch_solver_scratch():
  solver = object.__new__(MetalDenseSolve)
  fake_dtype = object()

  class FakeTensor:
    def __init__(self, shape, dtype=fake_dtype):
      self.shape = shape
      self.dtype = dtype

    @property
    def device(self):
      return SimpleNamespace(type="mps")

    def is_contiguous(self):
      return True

  solver._torch = SimpleNamespace(Tensor=FakeTensor, float32=fake_dtype)
  solver._device = SimpleNamespace(__str__=lambda _: "mps")
  solver.batch_size = 2
  solver.nv = 3
  solver.nrhs = 1
  solver._factor = np.full(18, 7.0)
  with pytest.raises(ValueError, match="mass must have shape"):
    solver.run_device(FakeTensor((2, 3, 2)), FakeTensor((2, 3)))
  with pytest.raises(TypeError, match="float32"):
    solver.run_device(FakeTensor((2, 3, 3), object()), FakeTensor((2, 3)))
  np.testing.assert_array_equal(solver._factor, np.full(18, 7.0))


@pytest.mark.skipif(
    os.environ.get("MUJOCO_METAL_RUN_GPU") != "1",
    reason="requires explicit MUJOCO_METAL_RUN_GPU=1 and idle GPU",
)
def test_batched_multiple_rhs_matches_numpy_and_keeps_mass_immutable():
  import torch

  rng = np.random.default_rng(921)
  batch, nv, nrhs = 3, 7, 4
  raw = rng.normal(size=(batch, nv, nv)).astype(np.float32)
  matrices = raw @ np.swapaxes(raw, 1, 2)
  matrices += np.eye(nv, dtype=np.float32)[None] * 0.25
  right = rng.normal(size=(batch, nv, nrhs)).astype(np.float32)
  original = matrices.copy()
  stage = MetalDenseSolve(nv, batch, nrhs)
  mass_gpu = torch.from_numpy(matrices).to("mps")
  rhs_gpu = torch.from_numpy(right).to("mps")
  actual, status = stage.run_device(
      mass_gpu, rhs_gpu
  )
  np.testing.assert_array_equal(status.cpu().numpy(), np.zeros(batch, np.int32))
  expected = np.stack(
      [np.linalg.solve(matrices[i], right[i]) for i in range(batch)]
  )
  np.testing.assert_allclose(
      actual.cpu().numpy(), expected, rtol=2e-5, atol=2e-6
  )
  np.testing.assert_array_equal(matrices, original)
  np.testing.assert_array_equal(mass_gpu.cpu().numpy(), original)

  next_solution, next_status = stage.run_device(
      mass_gpu,
      torch.from_numpy(2.0 * right).to("mps"),
  )
  assert next_solution.data_ptr() == actual.data_ptr()
  assert next_status.data_ptr() == status.data_ptr()
  np.testing.assert_allclose(
      actual.cpu().numpy(), 2.0 * expected, rtol=2e-5, atol=2e-6
  )


@pytest.mark.skipif(
    os.environ.get("MUJOCO_METAL_RUN_GPU") != "1",
    reason="requires explicit MUJOCO_METAL_RUN_GPU=1 and idle GPU",
)
def test_scaled_33_dof_system_and_zero_dof_shapes():
  import torch

  rng = np.random.default_rng(932)
  nv = 33
  q, _ = np.linalg.qr(rng.normal(size=(nv, nv)))
  eigenvalues = np.geomspace(0.25, 25.0, nv)
  base_mass = ((q * eigenvalues) @ q.T).astype(np.float32)
  scales = np.array([1e-10, 1e10], dtype=np.float32)
  mass = scales[:, None, None] * base_mass[None]
  rhs = scales[:, None] * rng.normal(size=(2, nv)).astype(np.float32)
  actual, status = MetalDenseSolve(nv, 2).run_device(
      torch.from_numpy(mass).to("mps"), torch.from_numpy(rhs).to("mps")
  )
  np.testing.assert_array_equal(status.cpu().numpy(), [0, 0])
  actual_host = actual.cpu().numpy()
  for world in range(2):
    residual = mass[world] @ actual_host[world] - rhs[world]
    normalized = np.linalg.norm(residual) / (
        np.linalg.norm(mass[world]) * np.linalg.norm(actual_host[world])
        + np.linalg.norm(rhs[world])
    )
    assert normalized < 2e-6

  empty, empty_status = MetalDenseSolve(0, 2).run_device(
      torch.empty((2, 0, 0), dtype=torch.float32, device="mps"),
      torch.empty((2, 0), dtype=torch.float32, device="mps"),
  )
  assert tuple(empty.shape) == (2, 0)
  np.testing.assert_array_equal(empty_status.cpu().numpy(), [0, 0])


@pytest.mark.skipif(
    os.environ.get("MUJOCO_METAL_RUN_GPU") != "1",
    reason="requires explicit MUJOCO_METAL_RUN_GPU=1 and idle GPU",
)
def test_bad_world_status_isolated_and_solution_is_zero():
  import torch

  mass = np.array(
      [
          [[2, 0], [0, 3]],
          [[1, 0.1], [0, 1]],
          [[1, 2], [2, 1]],
          [[1, 0], [0, np.nan]],
      ],
      dtype=np.float32,
  )
  rhs = np.ones((4, 2), dtype=np.float32)
  rhs[0, 0] = np.inf
  original = mass.copy()
  actual, status = MetalDenseSolve(2, 4).run_device(
      torch.from_numpy(mass).to("mps"), torch.from_numpy(rhs).to("mps")
  )
  np.testing.assert_array_equal(status.cpu().numpy(), [1, 2, 3, 1])
  np.testing.assert_array_equal(actual.cpu().numpy(), np.zeros((4, 2)))
  np.testing.assert_array_equal(mass, original)

  valid_and_invalid = np.stack([np.diag([2.0, 4.0]), np.diag([1.0, -1.0])])
  mixed, mixed_status = MetalDenseSolve(2, 2).run_device(
      torch.from_numpy(valid_and_invalid.astype(np.float32)).to("mps"),
      torch.ones((2, 2), dtype=torch.float32, device="mps"),
  )
  np.testing.assert_array_equal(mixed_status.cpu().numpy(), [0, 3])
  np.testing.assert_allclose(mixed.cpu().numpy()[0], [0.5, 0.25])
  np.testing.assert_array_equal(mixed.cpu().numpy()[1], [0, 0])


@pytest.mark.skipif(
    os.environ.get("MUJOCO_METAL_RUN_GPU") != "1",
    reason="requires explicit MUJOCO_METAL_RUN_GPU=1 and idle GPU",
)
def test_singular_and_intermediate_breakdown_status():
  import torch

  mass = np.array(
      [
          [[1, 1], [1, 1]],
          [[1, 1e20], [1e20, 2e38]],
      ],
      dtype=np.float32,
  )
  rhs = np.ones((2, 2), dtype=np.float32)
  actual, status = MetalDenseSolve(2, 2).run_device(
      torch.from_numpy(mass).to("mps"),
      torch.from_numpy(rhs).to("mps"),
  )
  np.testing.assert_array_equal(status.cpu().numpy(), [3, 4])
  np.testing.assert_array_equal(actual.cpu().numpy(), np.zeros((2, 2)))
