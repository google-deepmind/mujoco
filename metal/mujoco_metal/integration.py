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

"""Native MPS semi-implicit Euler position, velocity, and time update."""

from pathlib import Path
import math
import numbers
import struct

import numpy as np

from mujoco_metal.model import ModelDescriptor
from mujoco_metal.model import snapshot_descriptor

_SHADER = Path(__file__).parent / "shaders" / "integration.metal"
_JOINT_FREE = 0
_JOINT_BALL = 1
_JOINT_SLIDE = 2
_JOINT_HINGE = 3


class MetalEulerIntegration:
  """Reusable native MPS semi-implicit Euler stage for a fixed model/batch.

  ``run_device(qpos, qvel, qacc, time, solve_status)`` consumes contiguous
  float32 MPS states shaped ``[B,nq]``, ``[B,nv]``, ``[B,nv]``, ``[B]`` and an
  int32 status vector. It returns borrowed ``(qpos, qvel, time, status)`` MPS
  views backed by reusable output storage; each returned view is overwritten
  by the next call. A nonzero solve status is propagated unchanged and that
  row is left untouched. Integration failures use 10 for nonfinite input, 11
  for a zero quaternion, and 12 for nonfinite arithmetic. Failed rows preserve
  their input state. Rows commit independently after the full candidate state
  passes validation.

  The velocity update is ``qvel += dt*qacc``. Position integration follows
  MuJoCo 3.10 for hinge, slide, ball and free joints, using the updated
  velocity. Construction validates and snapshots model constants, initializes
  MPS, and compiles the shader. It does not read state values back to the host.
  """

  def __init__(self, model: ModelDescriptor, batch_size: int, timestep: float):
    if not isinstance(model, ModelDescriptor):
      raise TypeError("model must be a ModelDescriptor")
    if isinstance(batch_size, bool) or not isinstance(batch_size, int):
      raise TypeError("batch_size must be an integer")
    if batch_size <= 0:
      raise ValueError("batch_size must be positive")
    if isinstance(timestep, bool) or not isinstance(timestep, numbers.Real):
      raise TypeError("timestep must be a real number")
    dt = float(timestep)
    if not math.isfinite(dt) or dt <= 0:
      raise ValueError("timestep must be finite and positive")
    try:
      dt = struct.unpack("f", struct.pack("f", dt))[0]
    except (OverflowError, struct.error) as error:
      raise ValueError(
          "timestep must be representable as finite float32"
      ) from error
    if not math.isfinite(dt) or dt <= 0:
      raise ValueError(
          "timestep must be representable as positive finite float32"
      )

    self.model = snapshot_descriptor(model)
    if np.any(
        ~np.isin(
            self.model.jnt_type,
            (_JOINT_FREE, _JOINT_BALL, _JOINT_SLIDE, _JOINT_HINGE),
        )
    ):
      raise ValueError(
          "integration supports only free, ball, slide, and hinge joints"
      )
    self.batch_size = batch_size
    self.dt = dt
    self.nq = self.model.nq
    self.nv = self.model.nv
    self.njnt = self.model.njnt
    max_index = (1 << 32) - 1
    if any(
        value > (1 << 31) - 1
        for value in (self.nq, self.nv, self.njnt, batch_size)
    ):
      raise ValueError("dimensions must fit the shader's signed 32-bit ABI")
    if batch_size * max(self.nq, self.nv) > max_index:
      raise ValueError("state storage dimensions exceed shader indexing range")

    import torch

    if not torch.backends.mps.is_available():
      raise RuntimeError("PyTorch MPS is unavailable")
    if not hasattr(torch.mps, "compile_shader"):
      raise RuntimeError("PyTorch does not provide torch.mps.compile_shader")
    self._torch = torch
    self._device = torch.device("mps")
    self._library = torch.mps.compile_shader(_SHADER.read_text())
    self._kernel = self._library.semi_implicit_euler
    self._joint_type = self._device_array(self.model.jnt_type)
    self._joint_qposadr = self._device_array(self.model.jnt_qposadr)
    self._joint_dofadr = self._device_array(self.model.jnt_dofadr)
    self._dims = torch.tensor(
        [self.nq, self.nv, self.njnt, batch_size],
        dtype=torch.int32,
        device=self._device,
    )
    self._dt = torch.tensor([dt], dtype=torch.float32, device=self._device)
    self._empty_input = torch.zeros(1, dtype=torch.float32, device=self._device)
    self._candidate_qpos = torch.empty(
        max(batch_size * self.nq, 1), dtype=torch.float32, device=self._device
    )
    self._candidate_qvel = torch.empty(
        max(batch_size * self.nv, 1), dtype=torch.float32, device=self._device
    )
    self._candidate_time = torch.empty(
        batch_size, dtype=torch.float32, device=self._device
    )
    self._output_qpos = torch.empty_like(self._candidate_qpos)
    self._output_qvel = torch.empty_like(self._candidate_qvel)
    self._output_time = torch.empty_like(self._candidate_time)
    self._output_status = torch.empty(
        batch_size, dtype=torch.int32, device=self._device
    )

  def _device_array(self, value):
    torch = self._torch
    if value.size == 0:
      return torch.zeros(1, dtype=torch.int32, device=self._device)
    return torch.from_numpy(np.array(value, dtype=np.int32, copy=True)).to(
        self._device
    )

  def _validate_tensor(self, tensor, name, shape, dtype):
    torch = self._torch
    if not isinstance(tensor, torch.Tensor):
      raise TypeError(f"{name} must be a torch.Tensor")
    if tensor.dtype != dtype:
      raise TypeError(f"{name} must have dtype {dtype}")
    if tuple(tensor.shape) != shape:
      raise ValueError(
          f"{name} must have shape {shape}, got {tuple(tensor.shape)}"
      )
    if tensor.device.type != "mps":
      raise ValueError(f"{name} must be on {self._device}")
    if not tensor.is_contiguous():
      raise ValueError(f"{name} must be contiguous")

  def run_device(self, qpos, qvel, qacc, time, solve_status):
    """Return candidate states, preserving any row that fails validation."""
    batch = self.batch_size
    self._validate_tensor(qpos, "qpos", (batch, self.nq), self._torch.float32)
    self._validate_tensor(qvel, "qvel", (batch, self.nv), self._torch.float32)
    self._validate_tensor(qacc, "qacc", (batch, self.nv), self._torch.float32)
    self._validate_tensor(time, "time", (batch,), self._torch.float32)
    self._validate_tensor(
        solve_status, "solve_status", (batch,), self._torch.int32
    )

    qpos_buffer = qpos.reshape(-1) if self.nq else self._empty_input
    qvel_buffer = qvel.reshape(-1) if self.nv else self._empty_input
    qacc_buffer = qacc.reshape(-1) if self.nv else self._empty_input
    self._kernel(
        qpos_buffer,
        qvel_buffer,
        qacc_buffer,
        time,
        solve_status,
        self._joint_type,
        self._joint_qposadr,
        self._joint_dofadr,
        self._candidate_qpos,
        self._candidate_qvel,
        self._candidate_time,
        self._output_qpos,
        self._output_qvel,
        self._output_time,
        self._output_status,
        self._dims,
        self._dt,
        threads=(batch,),
        group_size=(1,),
    )
    qpos_output = self._output_qpos[: batch * self.nq].reshape(batch, self.nq)
    qvel_output = self._output_qvel[: batch * self.nv].reshape(batch, self.nv)
    return qpos_output, qvel_output, self._output_time, self._output_status
