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

"""Owned batched device state and host checkpoint/reset boundaries."""

from dataclasses import dataclass
import hashlib
import math

import mujoco
import numpy as np

from mujoco_metal.lifecycle import _fingerprint
from mujoco_metal.model import ModelDescriptor
from mujoco_metal.model import load_model
from mujoco_metal.model import snapshot_descriptor
from mujoco_metal.stepping import SteppingProfile
from mujoco_metal.stepping import TARGET_MUJOCO_VERSION
from mujoco_metal.stepping import validate_stepping_profile


def _freeze_float32(value, shape, name):
  array = np.asarray(value)
  if array.shape != shape or array.dtype.kind not in "fiu":
    raise ValueError(f"{name} must be numeric with shape {shape}")
  with np.errstate(over="ignore", under="ignore", invalid="ignore"):
    array = np.asarray(array, dtype=np.float32, order="C")
  if not np.all(np.isfinite(array)):
    raise ValueError(f"{name} must be finite and representable as float32")
  return np.frombuffer(array.tobytes(), dtype=np.float32).reshape(shape)


def _state_fingerprint(profile):
  digest = hashlib.sha256(b"mujoco-metal-device-state-v1\0")
  for value in (
      TARGET_MUJOCO_VERSION,
      profile.name,
      profile.model_fingerprint,
      profile.descriptor_fingerprint,
      repr(float(profile.timestep)),
      repr(profile.nq),
      repr(profile.nv),
      repr(profile.joint_types),
      repr(profile.supported),
      repr(profile.irrelevant),
      repr(profile.rejected),
  ):
    digest.update(value.encode("utf-8"))
    digest.update(b"\0")
  return digest.hexdigest()


@dataclass(frozen=True)
class StateSnapshot:
  """Detached float32 checkpoint tied to one model/profile and batch size."""

  model_fingerprint: str
  profile_fingerprint: str
  timestep: float
  nq: int
  nv: int
  batch_size: int
  qpos: np.ndarray
  qvel: np.ndarray
  qacc: np.ndarray
  time: np.ndarray
  status: np.ndarray
  schema_version: int = 1

  def __post_init__(self):
    if self.schema_version != 1:
      raise ValueError("unsupported state snapshot schema")
    if self.batch_size <= 0 or self.nq < 0 or self.nv < 0:
      raise ValueError("invalid state snapshot dimensions")
    if not math.isfinite(float(self.timestep)) or self.timestep <= 0:
      raise ValueError("snapshot timestep must be finite and positive")
    batch = self.batch_size
    object.__setattr__(
        self, "qpos", _freeze_float32(self.qpos, (batch, self.nq), "qpos")
    )
    object.__setattr__(
        self, "qvel", _freeze_float32(self.qvel, (batch, self.nv), "qvel")
    )
    object.__setattr__(
        self, "qacc", _freeze_float32(self.qacc, (batch, self.nv), "qacc")
    )
    object.__setattr__(
        self, "time", _freeze_float32(self.time, (batch,), "time")
    )
    raw_status = np.asarray(self.status)
    if raw_status.shape != (batch,) or raw_status.dtype.kind not in "iu":
      raise ValueError("status must be an integer vector matching batch size")
    if np.any(raw_status < np.iinfo(np.int32).min) or np.any(
        raw_status > np.iinfo(np.int32).max
    ):
      raise ValueError("status values must fit in int32")
    status = np.asarray(raw_status, dtype=np.int32, order="C")
    frozen = np.frombuffer(status.tobytes(), dtype=np.int32).reshape((batch,))
    object.__setattr__(self, "status", frozen)


class DeviceState:
  """Own persistent batched state tensors for one validated stepping profile.

  The default device is MPS. ``device='cpu'`` is provided for host-only state
  lifecycle verification; it does not enable CPU physics stepping. Device time
  uses float32, so its resolution decreases as elapsed time grows. Seeded
  randomization may be prepared as host arrays and passed to ``reset`` outside
  the step loop.
  """

  def __init__(
      self,
      model,
      profile: SteppingProfile,
      batch_size: int,
      qpos=None,
      qvel=None,
      device="mps",
  ):
    if isinstance(batch_size, (bool, np.bool_)) or not isinstance(
        batch_size, (int, np.integer)
    ) or batch_size <= 0:
      raise ValueError("batch_size must be a positive integer")
    if not isinstance(profile, SteppingProfile):
      raise TypeError("profile must be a validated SteppingProfile")
    if mujoco.__version__ != TARGET_MUJOCO_VERSION:
      raise RuntimeError(
          f"requires MuJoCo {TARGET_MUJOCO_VERSION}; found {mujoco.__version__}"
      )
    if isinstance(model, mujoco.MjModel):
      validated = validate_stepping_profile(
          model, profile.timestep, profile=profile.name
      )
      descriptor = load_model(model)
      if validated != profile:
        raise ValueError("profile does not match the compiled model")
      model_fingerprint = validated.model_fingerprint
    elif isinstance(model, ModelDescriptor):
      descriptor = model
      model_fingerprint = profile.model_fingerprint
      if _fingerprint(descriptor) != profile.descriptor_fingerprint:
        raise ValueError("profile does not match the immutable model descriptor")
      if (descriptor.nq, descriptor.nv) != (profile.nq, profile.nv):
        raise ValueError("profile dimensions do not match model descriptor")
      if tuple(int(x) for x in descriptor.jnt_type) != profile.joint_types:
        raise ValueError("profile joint types do not match model descriptor")
      descriptor = snapshot_descriptor(descriptor)
    else:
      raise TypeError("model must be an MjModel or immutable ModelDescriptor")

    self._model = descriptor
    self.profile = profile
    self.batch_size = int(batch_size)
    self._model_fingerprint = model_fingerprint
    self._profile_fingerprint = _state_fingerprint(profile)

    initial_qpos = np.broadcast_to(
        descriptor.qpos0, (self.batch_size, descriptor.nq)
    ).copy()
    initial_qvel = np.zeros((self.batch_size, descriptor.nv), dtype=np.float64)
    if qpos is not None:
      initial_qpos = self._host_values(qpos, initial_qpos.shape, "qpos")
    if qvel is not None:
      initial_qvel = self._host_values(qvel, initial_qvel.shape, "qvel")
    initial_qpos = self._validate_qpos(
        self._host_values(initial_qpos, initial_qpos.shape, "qpos")
    )
    initial_qvel = self._host_values(
        initial_qvel, initial_qvel.shape, "qvel"
    )

    import torch

    self._torch = torch
    try:
      self._device = torch.device(device)
    except (TypeError, RuntimeError) as error:
      raise ValueError(f"invalid state device: {device!r}") from error
    if self._device.type == "mps" and not torch.backends.mps.is_available():
      raise RuntimeError("MPS is unavailable; pass device='cpu' only for state testing")
    if self._device.type not in ("mps", "cpu"):
      raise ValueError("device state supports only MPS or explicit CPU state tests")

    self._qpos = torch.as_tensor(
        initial_qpos, dtype=torch.float32, device=self._device
    ).clone()
    self._qvel = torch.as_tensor(
        initial_qvel, dtype=torch.float32, device=self._device
    ).clone()
    self._qacc = torch.zeros(
        (self.batch_size, descriptor.nv), dtype=torch.float32, device=self._device
    )
    self._time = torch.zeros(
        (self.batch_size,), dtype=torch.float32, device=self._device
    )
    self._status = torch.zeros(
        (self.batch_size,), dtype=torch.int32, device=self._device
    )
    self._generation = 0

  @staticmethod
  def _host_values(value, shape, name):
    array = np.asarray(value)
    if array.shape != shape or array.dtype.kind not in "fiu":
      raise ValueError(f"{name} must be numeric with shape {shape}")
    with np.errstate(over="ignore", under="ignore", invalid="ignore"):
      array = np.asarray(array, dtype=np.float32, order="C")
    if not np.all(np.isfinite(array)):
      raise ValueError(f"{name} must be finite and representable as float32")
    return array.copy()

  def _validate_qpos(self, qpos):
    for joint, kind in enumerate(self._model.jnt_type):
      start = int(self._model.jnt_qposadr[joint])
      if int(kind) == int(mujoco.mjtJoint.mjJNT_FREE):
        quat_slice = slice(start + 3, start + 7)
      elif int(kind) == int(mujoco.mjtJoint.mjJNT_BALL):
        quat_slice = slice(start, start + 4)
      else:
        continue
      norms = np.linalg.norm(qpos[:, quat_slice], axis=1)
      if np.any(~np.isfinite(norms)) or np.any(np.abs(norms - 1) > 1e-5):
        raise ValueError("qpos free/ball quaternions must be unit length")
    return qpos

  @property
  def device(self):
    return str(self._device)

  @property
  def generation(self):
    return self._generation

  @property
  def qpos(self):
    return self._qpos.detach().clone()

  @property
  def qvel(self):
    return self._qvel.detach().clone()

  @property
  def qacc(self):
    return self._qacc.detach().clone()

  @property
  def time(self):
    return self._time.detach().clone()

  @property
  def status(self):
    return self._status.detach().clone()

  def _env_ids(self, env_ids):
    if env_ids is None:
      return np.arange(self.batch_size, dtype=np.int64)
    raw = np.asarray(env_ids)
    if raw.size == 0 and raw.ndim == 1:
      return np.empty((0,), dtype=np.int64)
    if raw.dtype.kind not in "iu":
      raise ValueError("env_ids must contain integers")
    ids = raw.astype(np.int64, copy=False)
    if ids.ndim != 1 or np.unique(ids).size != ids.size:
      raise ValueError("env_ids must be a vector of unique environment indices")
    if np.any(ids < 0) or np.any(ids >= self.batch_size):
      raise ValueError("env_ids are out of range")
    return ids

  def reset(self, env_ids=None, qpos=None, qvel=None):
    """Reset selected rows atomically from checked host arrays or model defaults."""
    ids = self._env_ids(env_ids)
    if not ids.size:
      return self._generation
    count = ids.size
    pos_default = np.broadcast_to(
        self._model.qpos0, (count, self._model.nq)
    ).copy()
    vel_default = np.zeros((count, self._model.nv), dtype=np.float32)
    pos = pos_default if qpos is None else self._host_values(
        qpos, pos_default.shape, "qpos"
    )
    vel = vel_default if qvel is None else self._host_values(
        qvel, vel_default.shape, "qvel"
    )
    pos = self._validate_qpos(pos)

    index = self._torch.as_tensor(ids, dtype=self._torch.int64, device=self._device)
    pos_tensor = self._torch.as_tensor(
        pos, dtype=self._torch.float32, device=self._device
    )
    vel_tensor = self._torch.as_tensor(
        vel, dtype=self._torch.float32, device=self._device
    )
    zero_acc = self._torch.zeros_like(vel_tensor)
    zero_time = self._torch.zeros(
        (count,), dtype=self._torch.float32, device=self._device
    )
    zero_status = self._torch.zeros(
        (count,), dtype=self._torch.int32, device=self._device
    )

    next_qpos = self._qpos.clone()
    next_qvel = self._qvel.clone()
    next_qacc = self._qacc.clone()
    next_time = self._time.clone()
    next_status = self._status.clone()
    next_qpos.index_copy_(0, index, pos_tensor)
    next_qvel.index_copy_(0, index, vel_tensor)
    next_qacc.index_copy_(0, index, zero_acc)
    next_time.index_copy_(0, index, zero_time)
    next_status.index_copy_(0, index, zero_status)
    self._qpos, self._qvel = next_qpos, next_qvel
    self._qacc, self._time, self._status = next_qacc, next_time, next_status
    self._generation += 1
    return self._generation

  def snapshot(self):
    """Copy all state to an immutable host checkpoint outside the step loop."""
    return StateSnapshot(
        model_fingerprint=self._model_fingerprint,
        profile_fingerprint=self._profile_fingerprint,
        timestep=self.profile.timestep,
        nq=self._model.nq,
        nv=self._model.nv,
        batch_size=self.batch_size,
        qpos=self._qpos.detach().cpu().numpy(),
        qvel=self._qvel.detach().cpu().numpy(),
        qacc=self._qacc.detach().cpu().numpy(),
        time=self._time.detach().cpu().numpy(),
        status=self._status.detach().cpu().numpy(),
    )

  def restore(self, snapshot):
    """Restore a matching checkpoint only after validating every field."""
    if not isinstance(snapshot, StateSnapshot):
      raise TypeError("snapshot must be a StateSnapshot")
    if snapshot.schema_version != 1:
      raise ValueError("unsupported state snapshot schema")
    if (
        snapshot.model_fingerprint != self._model_fingerprint
        or snapshot.profile_fingerprint != self._profile_fingerprint
        or snapshot.timestep != self.profile.timestep
        or snapshot.nq != self._model.nq
        or snapshot.nv != self._model.nv
        or snapshot.batch_size != self.batch_size
    ):
      raise ValueError("snapshot model, profile, timestep, or dimensions do not match")
    qpos = self._validate_qpos(
        self._host_values(snapshot.qpos, (self.batch_size, self._model.nq), "qpos")
    )
    qvel = self._host_values(
        snapshot.qvel, (self.batch_size, self._model.nv), "qvel"
    )
    qacc = self._host_values(
        snapshot.qacc, (self.batch_size, self._model.nv), "qacc"
    )
    time = self._host_values(snapshot.time, (self.batch_size,), "time")
    if np.any(time < 0):
      raise ValueError("snapshot time must be nonnegative")
    status = np.asarray(snapshot.status)
    if status.shape != (self.batch_size,) or status.dtype.kind not in "iu":
      raise ValueError("snapshot status has an invalid shape or dtype")
    if np.any(status < np.iinfo(np.int32).min) or np.any(
        status > np.iinfo(np.int32).max
    ):
      raise ValueError("snapshot status values must fit in int32")

    tensors = (
        self._torch.as_tensor(qpos, dtype=self._torch.float32, device=self._device),
        self._torch.as_tensor(qvel, dtype=self._torch.float32, device=self._device),
        self._torch.as_tensor(qacc, dtype=self._torch.float32, device=self._device),
        self._torch.as_tensor(time, dtype=self._torch.float32, device=self._device),
        self._torch.as_tensor(
            np.asarray(status, dtype=np.int32).copy(),
            dtype=self._torch.int32,
            device=self._device,
        ),
    )
    self._qpos, self._qvel, self._qacc, self._time, self._status = tensors
    self._generation += 1
    return self._generation
