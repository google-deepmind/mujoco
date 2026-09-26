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

"""Explicit opt-in Metal dense mass-matrix stage; MPS output stays on device."""

from pathlib import Path

import numpy as np

from mujoco_metal.metal_kinematics import _prepare_host_arrays
from mujoco_metal.metal_kinematics import MetalKinematics
from mujoco_metal.model import ModelDescriptor

_SHADER = Path(__file__).parent / "shaders" / "smooth_mass.metal"
_BIAS_SHADER = Path(__file__).parent / "shaders" / "smooth_bias.metal"
_DEVICE_ARRAYS = (
    "body_parentid",
    "body_rootid",
    "body_jntadr",
    "body_jntnum",
    "body_dofadr",
    "body_dofnum",
    "dof_parentid",
    "dof_bodyid",
    "jnt_type",
    "jnt_dofadr",
    "body_mass",
    "body_inertia",
    "dof_armature",
    "gravity",
)


class MetalSmoothDynamics:
  """Batched native MPS smooth inertial dynamics stage.

  ``mass_matrix(qpos_batch)`` returns an MPS tensor with shape ``[B,nv,nv]``.
  ``run(qpos_batch, qvel_batch)`` returns the mass matrix and inertial/gravity
  bias as MPS tensors. It does not compute actuation, passive forces, contacts,
  constraints, or physics steps. Construction explicitly initializes MPS and
  compiles shaders.
  """

  def __init__(self, model: ModelDescriptor):
    if model.nu:
      raise ValueError("Metal smooth stage does not support actuators")
    if np.any(model.tendon_armature != 0):
      raise ValueError("Metal smooth stage does not support tendon armature")
    host = _prepare_host_arrays(model)
    self._fk = MetalKinematics(model)
    self.model = self._fk.model
    torch = self._fk._torch
    self._torch = torch
    self._library = torch.mps.compile_shader(_SHADER.read_text())
    self._kernel = self._library.dense_mass_matrix
    self._bias_library = torch.mps.compile_shader(_BIAS_SHADER.read_text())
    self._bias_kernel = self._bias_library.smooth_bias
    self._arrays = {
        name: torch.from_numpy(host[name]).to(self._fk._device)
        for name in _DEVICE_ARRAYS
    }

  def _compute_mass_matrix(self, qpos_batch):
    source = np.asarray(qpos_batch, dtype=np.float64)
    if source.ndim == 1:
      source = source[None, :]
    if (
        source.ndim != 2
        or source.shape[0] == 0
        or source.shape[1] != self.model.nq
        or not np.all(np.isfinite(source))
    ):
      raise ValueError(
          f"qpos must be finite with shape (batch, {self.model.nq}) and batch > 0"
      )
    qpos32 = np.asarray(source, dtype=np.float32)
    if not np.all(np.isfinite(qpos32)):
      raise ValueError("qpos cannot be represented as finite float32")

    # FK validates and normalizes its own float32 input before allocating MPS.
    poses = self._fk.run(source)
    batch = source.shape[0]
    nv, nb = self.model.nv, self.model.nbody
    torch = self._torch
    output = torch.empty(
        max(batch * nv * nv, 1), dtype=torch.float32, device=self._fk._device
    )
    root_com = torch.empty(
        max(batch * nb * 3, 1), dtype=torch.float32, device=self._fk._device
    )
    cdof = torch.empty(
        max(batch * nv * 6, 1), dtype=torch.float32, device=self._fk._device
    )
    crb = torch.empty(
        max(batch * nb * 36, 1), dtype=torch.float32, device=self._fk._device
    )
    local_inertia = torch.empty(
        max(batch * nb * 36, 1), dtype=torch.float32, device=self._fk._device
    )
    arrays = self._arrays
    args = [
        arrays["body_parentid"],
        arrays["body_rootid"],
        arrays["body_jntadr"],
        arrays["body_jntnum"],
        arrays["dof_parentid"],
        arrays["dof_bodyid"],
        arrays["jnt_type"],
        arrays["jnt_dofadr"],
        arrays["body_mass"],
        arrays["body_inertia"],
        arrays["dof_armature"],
        poses["body_quat"].reshape(-1),
        poses["inertial_pos"].reshape(-1),
        poses["inertial_quat"].reshape(-1),
        poses["joint_anchor"].reshape(-1),
        poses["joint_axis"].reshape(-1),
        output,
        root_com,
        cdof,
        crb,
        torch.tensor(
            [nb, self.model.njnt, nv, batch],
            dtype=torch.int32,
            device=self._fk._device,
        ),
        local_inertia,
    ]
    if len(args) != 22:
      raise RuntimeError("native dense mass shader buffer ABI mismatch")
    self._kernel(*args, threads=(batch,), group_size=(1,))
    return {
        "mass_matrix": output[: batch * nv * nv].reshape(batch, nv, nv),
        "root_com": root_com,
        "cdof": cdof,
        "local_inertia": local_inertia,
        "poses": poses,
        "batch": batch,
    }

  def mass_matrix(self, qpos_batch):
    """Compute dense generalized inertia for a batch, returning an MPS tensor."""
    return self._compute_mass_matrix(qpos_batch)["mass_matrix"]

  def run(self, qpos_batch, qvel_batch):
    """Return batched mass matrices and inertial/gravity bias on the MPS device."""
    qpos = np.asarray(qpos_batch, dtype=np.float64)
    if qpos.ndim == 1:
      qpos = qpos[None, :]
    if (
        qpos.ndim != 2
        or qpos.shape[0] == 0
        or qpos.shape[1] != self.model.nq
        or not np.all(np.isfinite(qpos))
    ):
      raise ValueError(
          f"qpos must be finite with shape (batch, {self.model.nq})"
      )
    if not np.all(np.isfinite(np.asarray(qpos, dtype=np.float32))):
      raise ValueError("qpos cannot be represented as finite float32")
    qvel = np.asarray(qvel_batch, dtype=np.float64)
    if qvel.ndim == 1:
      qvel = qvel[None, :]
    if (
        qvel.ndim != 2
        or qvel.shape != (qpos.shape[0], self.model.nv)
        or qpos.shape[0] == 0
        or not np.all(np.isfinite(qvel))
    ):
      raise ValueError(
          f"qvel must be finite with shape (batch, {self.model.nv})"
      )
    qvel32 = np.asarray(qvel, dtype=np.float32)
    if not np.all(np.isfinite(qvel32)):
      raise ValueError("qvel cannot be represented as finite float32")

    computed = self._compute_mass_matrix(qpos)
    batch, nb, nv = computed["batch"], self.model.nbody, self.model.nv
    torch = self._torch
    qvel_flat = qvel32.reshape(-1)
    if qvel_flat.size == 0:
      qvel_flat = np.zeros(1, dtype=np.float32)
    qvel_tensor = torch.from_numpy(qvel_flat).to(self._fk._device)
    bias = torch.empty(
        max(batch * nv, 1), dtype=torch.float32, device=self._fk._device
    )
    cvel = torch.empty(
        max(batch * nb * 6, 1), dtype=torch.float32, device=self._fk._device
    )
    cdof_dot = torch.empty(
        max(batch * nv * 6, 1), dtype=torch.float32, device=self._fk._device
    )
    cacc = torch.empty(
        max(batch * nb * 6, 1), dtype=torch.float32, device=self._fk._device
    )
    body_force = torch.empty(
        max(batch * nb * 6, 1), dtype=torch.float32, device=self._fk._device
    )
    args = [
        self._arrays["body_parentid"],
        self._arrays["body_dofadr"],
        self._arrays["body_dofnum"],
        self._arrays["body_jntadr"],
        self._arrays["body_jntnum"],
        self._arrays["dof_bodyid"],
        self._arrays["jnt_type"],
        self._arrays["jnt_dofadr"],
        self._arrays["gravity"],
        computed["cdof"],
        computed["local_inertia"],
        torch.tensor(
            [self.model.disableflags],
            dtype=torch.int32,
            device=self._fk._device,
        ),
        qvel_tensor,
        cvel,
        cdof_dot,
        cacc,
        body_force,
        bias,
        torch.tensor(
            [nb, self.model.njnt, nv, batch],
            dtype=torch.int32,
            device=self._fk._device,
        ),
    ]
    if len(args) != 19:
      raise RuntimeError("native smooth bias shader buffer ABI mismatch")
    self._bias_kernel(*args, threads=(batch,), group_size=(1,))
    return {
        "mass_matrix": computed["mass_matrix"],
        "qfrc_bias": bias[: batch * nv].reshape(batch, nv),
    }
