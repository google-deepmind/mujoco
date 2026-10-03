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

"""Native MPS dense Cholesky solve for batched symmetric positive systems."""

from pathlib import Path

_SHADER = Path(__file__).parent / "shaders" / "smooth_solve.metal"


class MetalDenseSolve:
  """Reusable dense SPD factorization and solve stage on the MPS device.

  Construct with fixed ``nv``, ``batch_size`` and ``nrhs`` capacities. For one
  right-hand side, ``rhs`` and the solution have shape ``[B,nv]``. For multiple
  right-hand sides they have shape ``[B,nv,nrhs]``. ``run_device`` returns
  ``(solution, status)`` where status is an int32 MPS vector, zero on success.
  Status codes are 1 for nonfinite input, 2 for asymmetric mass, 3 for a
  non-positive Cholesky pivot, and 4 for nonfinite intermediate arithmetic.
  Failed rows return a zero solution; rows are independent. Output and factor
  storage are reused, so returned views are valid only until the next call.

  This primitive validates but never modifies its mass input. It does not add
  diagonal jitter, perform a host solve, or copy values back to the CPU.
  Construction explicitly initializes MPS and compiles its shader.
  """

  def __init__(self, nv: int, batch_size: int, nrhs: int = 1):
    for name, value in (("nv", nv), ("batch_size", batch_size), ("nrhs", nrhs)):
      if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{name} must be an integer")
    if nv < 0:
      raise ValueError("nv must be nonnegative")
    if batch_size <= 0:
      raise ValueError("batch_size must be positive")
    if nrhs <= 0:
      raise ValueError("nrhs must be positive")
    max_index = (1 << 32) - 1
    if any(value > (1 << 31) - 1 for value in (nv, batch_size, nrhs)):
      raise ValueError("dimensions must fit the shader's signed 32-bit ABI")
    if nv * nv > max_index or batch_size * nv * nv > max_index:
      raise ValueError("mass storage dimensions exceed shader indexing range")
    if batch_size * nv * nrhs > max_index:
      raise ValueError("RHS storage dimensions exceed shader indexing range")

    import torch

    if not torch.backends.mps.is_available():
      raise RuntimeError("PyTorch MPS is unavailable")
    if not hasattr(torch.mps, "compile_shader"):
      raise RuntimeError("PyTorch does not provide torch.mps.compile_shader")
    self._torch = torch
    self._device = torch.device("mps")
    self.nv = nv
    self.batch_size = batch_size
    self.nrhs = nrhs
    self._library = torch.mps.compile_shader(_SHADER.read_text())
    self._kernel = self._library.dense_spd_solve
    self._factor = torch.empty(
        max(batch_size * nv * nv, 1), dtype=torch.float32, device=self._device
    )
    self._solution = torch.empty(
        max(batch_size * nv * nrhs, 1), dtype=torch.float32, device=self._device
    )
    self._status = torch.empty(
        batch_size, dtype=torch.int32, device=self._device
    )
    self._empty_input = torch.zeros(1, dtype=torch.float32, device=self._device)
    self._dims = torch.tensor(
        [nv, batch_size, nrhs], dtype=torch.int32, device=self._device
    )

  def _validate_tensor(self, tensor, name, shape):
    torch = self._torch
    if not isinstance(tensor, torch.Tensor):
      raise TypeError(f"{name} must be a torch.Tensor")
    if tensor.dtype != torch.float32:
      raise TypeError(f"{name} must have dtype torch.float32")
    if tuple(tensor.shape) != shape:
      raise ValueError(
          f"{name} must have shape {shape}, got {tuple(tensor.shape)}"
      )
    if tensor.device.type != "mps":
      raise ValueError(f"{name} must be on {self._device}")
    if not tensor.is_contiguous():
      raise ValueError(f"{name} must be contiguous")

  def run_device(self, mass, rhs):
    """Solve ``mass @ solution = rhs`` for all fixed-capacity batch rows.

    ``mass`` is ``[B,nv,nv]``. ``rhs`` is ``[B,nv]`` when ``nrhs==1`` and
    ``[B,nv,nrhs]`` otherwise. Input values are checked in the shader so
    failures are reported per row without a synchronization or host readback.
    """
    self._validate_tensor(mass, "mass", (self.batch_size, self.nv, self.nv))
    rhs_shape = (
        (self.batch_size, self.nv)
        if self.nrhs == 1
        else (self.batch_size, self.nv, self.nrhs)
    )
    self._validate_tensor(rhs, "rhs", rhs_shape)

    mass_buffer = mass.reshape(-1)
    rhs_buffer = rhs.reshape(-1)
    if self.nv == 0:
      mass_buffer = self._empty_input
      rhs_buffer = self._empty_input
    self._kernel(
        mass_buffer,
        rhs_buffer,
        self._factor,
        self._solution,
        self._status,
        self._dims,
        threads=(self.batch_size,),
        group_size=(1,),
    )
    if self.nrhs == 1:
      solution = self._solution[: self.batch_size * self.nv].reshape(
          self.batch_size, self.nv
      )
    else:
      solution = self._solution[
          : self.batch_size * self.nv * self.nrhs
      ].reshape(self.batch_size, self.nv, self.nrhs)
    return solution, self._status
