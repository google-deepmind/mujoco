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

"""Explicitly opt-in Torch MPS launcher for the kinematics-only stage."""

from pathlib import Path

import numpy as np

from mujoco_metal.model import _validate_lowered
from mujoco_metal.model import ModelDescriptor
from mujoco_metal.model import snapshot_descriptor

_SHADER = Path(__file__).parent / "shaders" / "kinematics.metal"


def _prepare_host_arrays(model: ModelDescriptor):
  """Validate and pack float32/int32 constants without initializing a device."""
  counts = {
      "nq": model.nq,
      "nv": model.nv,
      "nbody": model.nbody,
      "njnt": model.njnt,
      "ngeom": model.ngeom,
      "nsite": model.nsite,
      "disableflags": model.disableflags,
  }
  names = (
      "body_rootid",
      "body_dofadr",
      "body_dofnum",
      "body_subtreemass",
      "dof_parentid",
      "dof_bodyid",
      "dof_jntid",
      "gravity",
      "body_parentid",
      "body_jntadr",
      "body_jntnum",
      "body_pos",
      "body_quat",
      "body_ipos",
      "body_iquat",
      "body_mass",
      "body_inertia",
      "dof_armature",
      "jnt_type",
      "jnt_qposadr",
      "jnt_dofadr",
      "jnt_bodyid",
      "jnt_pos",
      "jnt_axis",
      "qpos0",
      "geom_bodyid",
      "geom_type",
      "geom_size",
      "geom_pos",
      "geom_quat",
      "site_bodyid",
      "site_pos",
      "site_quat",
  )
  values = {name: getattr(model, name) for name in names}
  _validate_lowered(counts, values)
  if model.nmocap:
    raise ValueError("Metal kinematics does not support mocap inputs")
  host = {}
  for name, value in values.items():
    dtype = np.int32 if np.asarray(value).dtype.kind in "iu" else np.float32
    converted = np.array(value, dtype=dtype, copy=True).reshape(-1)
    if converted.dtype.kind == "f" and not np.all(np.isfinite(converted)):
      raise ValueError(f"{name} cannot be represented as finite float32")
    if name not in ("body_jntadr", "body_jntnum") and converted.size == 0:
      converted = np.zeros(1, dtype=dtype)
    host[name] = converted
  for name in ("body_quat", "body_iquat", "geom_quat", "site_quat"):
    if np.asarray(values[name]).size == 0:
      continue
    quat = host[name].reshape(-1, 4)
    norms = np.linalg.norm(quat.astype(np.float64), axis=1)
    if np.any(~np.isfinite(norms)) or np.any(norms <= 0):
      raise ValueError(
          f"{name} cannot be represented as nonzero float32 quaternions"
      )
  return host


def _shape_output(buffer, batch, count, width):
  """Drop dummy storage from zero-length GPU outputs before reshaping."""
  return buffer[: batch * count * width].reshape(batch, count, width)


class MetalKinematics:
  """Batched native MSL forward kinematics; construction initializes MPS."""

  def __init__(self, model: ModelDescriptor):
    host_arrays = _prepare_host_arrays(model)
    self.model = snapshot_descriptor(model)
    # Importing this module remains host-only; construction is the explicit
    # device capability boundary.
    import torch

    self._torch = torch
    if not torch.backends.mps.is_available():
      raise RuntimeError("PyTorch MPS is unavailable")
    if not hasattr(torch.mps, "compile_shader"):
      raise RuntimeError("PyTorch does not provide torch.mps.compile_shader")
    self._device = torch.device("mps")
    self._library = torch.mps.compile_shader(_SHADER.read_text())
    self._kernel = self._library.forward_kinematics
    self._arrays = {}
    for name, host in host_arrays.items():
      if name in (
          "body_jntadr",
          "body_jntnum",
          "body_rootid",
          "body_dofadr",
          "body_dofnum",
          "body_subtreemass",
          "dof_parentid",
          "dof_bodyid",
          "dof_jntid",
          "gravity",
          "jnt_dofadr",
          "body_mass",
          "body_inertia",
          "dof_armature",
          "geom_type",
          "geom_size",
      ):
        continue
      self._arrays[name] = torch.from_numpy(host).to(self._device)

  def run(self, qpos):
    """Compute full world poses for a CPU qpos batch; returns MPS tensors."""
    source = np.asarray(qpos, dtype=np.float64)
    if source.ndim == 1:
      source = source[None, :]
    if (
        source.ndim != 2
        or source.shape[1] != self.model.nq
        or source.shape[0] == 0
    ):
      raise ValueError(
          f"qpos must have shape (batch, {self.model.nq}) with batch > 0"
      )
    if not np.all(np.isfinite(source)):
      raise ValueError("qpos must be finite")
    q = np.array(source, dtype=np.float32, order="C", copy=True)
    if not np.all(np.isfinite(q)):
      raise ValueError("qpos cannot be represented as finite float32")
    for typ, qa in zip(self.model.jnt_type, self.model.jnt_qposadr):
      start = int(qa) + (3 if int(typ) == 0 else 0)
      width = 4 if int(typ) in (0, 1) else 0
      if width:
        norms = np.linalg.norm(
            q[:, start : start + width].astype(np.float64), axis=1
        )
        if not np.all(np.isfinite(norms)) or np.any(norms <= 0):
          raise ValueError(
              "free/ball quaternion must be finite and nonzero in float32"
          )
        q[:, start : start + width] = (
            q[:, start : start + width].astype(np.float64) / norms[:, None]
        ).astype(np.float32)
    torch = self._torch
    q_flat = q.reshape(-1)
    if q_flat.size == 0:
      q_flat = np.zeros(1, dtype=np.float32)
    q_tensor = torch.from_numpy(q_flat).to(self._device)
    shapes = {
        "body": (self.model.nbody,),
        "geom": (self.model.ngeom,),
        "site": (self.model.nsite,),
        "inertial": (self.model.nbody,),
    }
    outputs = {}
    for name, shape in shapes.items():
      count = source.shape[0] * shape[0]
      outputs[f"{name}_pos"] = torch.empty(
          max(count * 3, 1), dtype=torch.float32, device=self._device
      )
      outputs[f"{name}_quat"] = torch.empty(
          max(count * 4, 1), dtype=torch.float32, device=self._device
      )
    arrays = self._arrays
    args = [
        arrays[name]
        for name in (
            "body_parentid",
            "body_pos",
            "body_quat",
            "jnt_type",
            "jnt_qposadr",
            "jnt_bodyid",
            "jnt_pos",
            "jnt_axis",
            "qpos0",
        )
    ]
    args.extend([q_tensor, outputs["body_pos"], outputs["body_quat"]])
    args.extend(
        arrays[name] for name in ("geom_bodyid", "geom_pos", "geom_quat")
    )
    args.extend([outputs["geom_pos"], outputs["geom_quat"]])
    args.extend(
        arrays[name] for name in ("site_bodyid", "site_pos", "site_quat")
    )
    args.extend([outputs["site_pos"], outputs["site_quat"]])
    args.extend(
        [
            arrays["body_ipos"],
            arrays["body_iquat"],
            outputs["inertial_pos"],
            outputs["inertial_quat"],
        ]
    )
    dims = torch.tensor(
        [
            self.model.nq,
            self.model.nbody,
            self.model.njnt,
            self.model.ngeom,
            self.model.nsite,
            source.shape[0],
        ],
        dtype=torch.int32,
        device=self._device,
    )
    args.append(dims)
    self._kernel(*args, threads=(source.shape[0],), group_size=(1,))
    shaped = {}
    for kind, count in (
        ("body", self.model.nbody),
        ("geom", self.model.ngeom),
        ("site", self.model.nsite),
        ("inertial", self.model.nbody),
    ):
      shaped[f"{kind}_pos"] = _shape_output(
          outputs[f"{kind}_pos"], source.shape[0], count, 3
      )
      shaped[f"{kind}_quat"] = _shape_output(
          outputs[f"{kind}_quat"], source.shape[0], count, 4
      )
    return shaped
