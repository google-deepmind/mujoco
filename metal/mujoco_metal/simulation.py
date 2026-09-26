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

"""Owned batched state and checked stepping around the ported Metal pipeline."""

import copy

import numpy as np

from mujoco_metal import model as model_lib


def positive_int(value, name, maximum=None):
  if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
    raise ValueError(f"{name} must be a positive integer")
  if maximum is not None and value > maximum:
    raise ValueError(f"{name} must be <= {maximum}")
  return value


def validate_state(qpos, qvel, batch_size):
  """Validate reset/checkpoint data entirely before changing device state."""
  qpos = np.array(qpos, dtype=np.float32, copy=True)
  qvel = np.array(qvel, dtype=np.float32, copy=True)
  if qpos.shape != (batch_size, 21) or qvel.shape != (batch_size, 20):
    raise ValueError(
        f"Expected qpos ({batch_size}, 21), qvel ({batch_size}, 20)"
    )
  if not np.isfinite(qpos).all() or not np.isfinite(qvel).all():
    raise ValueError("State must be finite")
  if not np.allclose(
      np.linalg.norm(qpos[:, 3:7], axis=1), 1, atol=1e-5, rtol=0
  ):
    raise ValueError("Root quaternions must have unit norm")
  return qpos, qvel


def check_result(output, torch, *, require_convergence=False):
  """Reject invalid physics; optionally require PGS convergence as well."""
  for stage, status in (
      ("contacts", output.contact_overflow),
      ("constraints", output.assembly_overflow),
      ("factorization", output.cholesky_status),
      ("integration", output.integration_status),
  ):
    if bool((status != 0).any().item()):
      raise RuntimeError(f"Metal {stage} failed: {status.cpu().tolist()}")
  status = output.solver_status
  allowed = (
      (status == 0) if require_convergence else ((status == 0) | (status == 1))
  )
  if not bool(allowed.all().item()):
    raise RuntimeError(f"Metal solver failed: {status.cpu().tolist()}")
  if not bool(
      (
          torch.isfinite(output.qpos).all() & torch.isfinite(output.qvel).all()
      ).item()
  ):
    raise RuntimeError("Metal produced nonfinite state")


class Simulation:
  """Float32 MPS physics for the exact microduck-flat-v1 compiled profile.

  Each instance owns its state and scratch buffers. Calls must be serialized.
  ``step`` synchronizes to check numerical/overflow status before committing
  state. Diagnostic results alias scratch storage; clone them to keep a history.
  CPU snapshots contain physics state only, not a learner or optimizer.
  """

  def __init__(
      self,
      model=None,
      *,
      batch_size=1,
      contact_capacity=35,
      constraint_capacity=128,
      require_convergence=False,
  ):
    positive_int(batch_size, "batch_size")
    positive_int(contact_capacity, "contact_capacity", 35)
    positive_int(constraint_capacity, "constraint_capacity", 128)
    if constraint_capacity % 4:
      raise ValueError("constraint_capacity must be a multiple of 4")
    if not isinstance(require_convergence, bool):
      raise TypeError("require_convergence must be bool")
    self.require_convergence = require_convergence
    canonical = model_lib.load_canonical()
    if model is not None:
      model_lib.validate_model(model, canonical.model)
      canonical.model = copy.copy(model)
    self._model = canonical.model
    self._fingerprint = model_lib.model_fingerprint(self._model)
    self.batch_size = batch_size

    # Import Torch only when the caller explicitly constructs a GPU simulation.
    import torch

    from mujoco_metal._physics import RepresentativePhysicsSlice

    if not torch.backends.mps.is_available():
      raise RuntimeError("Metal physics requires Apple Silicon with Torch MPS")
    if torch.__version__.split("+")[0] != "2.9.1":
      raise RuntimeError(
          "This port requires torch==2.9.1 pending requalification"
      )
    self._torch = torch
    self._physics = RepresentativePhysicsSlice(
        batch_size=batch_size,
        nconmax=contact_capacity,
        canonical=canonical,
        autonomous_capacity=constraint_capacity,
    )
    self._qpos = torch.empty(
        (batch_size, 21), device="mps", dtype=torch.float32
    )
    self._qvel = torch.empty(
        (batch_size, 20), device="mps", dtype=torch.float32
    )
    self._damping = torch.tensor(
        self._model.dof_damping, device="mps", dtype=torch.float32
    )
    self._frictionloss = torch.tensor(
        self._model.dof_frictionloss[6:], device="mps", dtype=torch.float32
    )
    self._time = np.zeros(batch_size, dtype=np.float64)
    self.reset()

  @property
  def qpos(self):
    """Return a device copy of positions; use reset to replace owned state."""
    return self._qpos.clone()

  @property
  def qvel(self):
    """Return a device copy of velocities."""
    return self._qvel.clone()

  def reset(self, qpos=None, qvel=None, *, env_ids=None):
    """Reset all or selected rows; omitted state uses the standing keyframe.

    Arrays are CPU-compatible and must contain exactly the selected row count.
    Reset validates all rows before allocation/copy and resets their times.
    """
    ids = np.arange(self.batch_size) if env_ids is None else np.asarray(env_ids)
    if (
        ids.ndim != 1
        or ids.dtype.kind not in "iu"
        or len(np.unique(ids)) != len(ids)
    ):
      raise ValueError(
          "env_ids must be a one-dimensional array of unique integers"
      )
    if np.any(ids < 0) or np.any(ids >= self.batch_size):
      raise ValueError("env_ids out of range")
    if qpos is None:
      qpos = np.tile(self._model.key_qpos[0], (len(ids), 1))
    if qvel is None:
      qvel = np.zeros((len(ids), 20))
    qp, qv = validate_state(qpos, qvel, len(ids))
    torch = self._torch
    index = torch.tensor(ids.astype(np.int64), device="mps")
    positions = torch.tensor(qp, device="mps")
    velocities = torch.tensor(qv, device="mps")
    self._qpos[index] = positions
    self._qvel[index] = velocities
    self._time[ids] = 0

  def step(self, ctrl):
    """Apply direct motor torques for one fixed 5 ms step, returning diagnostics.

    ctrl must be a finite float32 MPS tensor shaped (batch_size, 14). Failed
    solves leave the owned state unchanged. Checking status incurs a GPU fence.
    """
    torch = self._torch
    if not isinstance(ctrl, torch.Tensor):
      raise TypeError("ctrl must be a Torch tensor")
    if (
        ctrl.shape != (self.batch_size, 14)
        or ctrl.dtype != torch.float32
        or ctrl.device.type != "mps"
    ):
      raise ValueError("ctrl must be float32 MPS with shape (batch_size, 14)")
    if not bool(torch.isfinite(ctrl).all().item()):
      raise ValueError("ctrl must be finite")
    output = self._physics.step_autonomous(
        self._qpos,
        self._qvel,
        ctrl=ctrl.contiguous(),
        dof_damping=self._damping,
        dof_frictionloss=self._frictionloss,
        dt=0.005,
        max_iters=100,
        tol=1e-5,
    )
    check_result(output, torch, require_convergence=self.require_convergence)
    self._qpos.copy_(output.qpos)
    self._qvel.copy_(output.qvel)
    self._time += 0.005
    return output

  def state_dict(self):
    """Return an independent CPU snapshot with a versioned model signature."""
    return {
        "format_version": 1,
        "profile": model_lib.PROFILE,
        "model_fingerprint": self._fingerprint,
        "batch_size": self.batch_size,
        "qpos": self._qpos.cpu().numpy().copy(),
        "qvel": self._qvel.cpu().numpy().copy(),
        "time": self._time.copy(),
    }

  def load_state_dict(self, state):
    """Restore compatible state after validating the complete snapshot."""
    for key, expected in (
        ("format_version", 1),
        ("profile", model_lib.PROFILE),
        ("model_fingerprint", self._fingerprint),
        ("batch_size", self.batch_size),
    ):
      if state.get(key) != expected:
        raise ValueError(f"Incompatible snapshot field: {key}")
    qp, qv = validate_state(state["qpos"], state["qvel"], self.batch_size)
    time = np.array(state["time"], dtype=np.float64, copy=True)
    if (
        time.shape != (self.batch_size,)
        or not np.isfinite(time).all()
        or np.any(time < 0)
    ):
      raise ValueError("Invalid snapshot time")
    self.reset(qp, qv)
    self._time[:] = time
