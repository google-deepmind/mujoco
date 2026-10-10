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

"""CPU-only model-constant and batched kinematics state lifecycle."""

from copy import copy
from dataclasses import dataclass
from dataclasses import fields
import hashlib
from pathlib import Path

import mujoco
import numpy as np

from mujoco_metal.model import load_model
from mujoco_metal.model import ModelDescriptor
from mujoco_metal.model import snapshot_descriptor


def _fingerprint(model):
  digest = hashlib.sha256(b"mujoco-metal-lifecycle-v1\0")
  if isinstance(model, mujoco.MjModel):
    buffer = np.empty(mujoco.mj_sizeModel(model), dtype=np.uint8)
    mujoco.mj_saveModel(model, buffer=buffer)
    digest.update(buffer.tobytes())
    return digest.hexdigest()
  for name in (
      "nq",
      "nv",
      "nu",
      "nmocap",
      "nbody",
      "njnt",
      "ngeom",
      "nsite",
      "ntendon",
      "disableflags",
  ):
    digest.update(int(getattr(model, name)).to_bytes(8, "little"))
  for item in fields(ModelDescriptor):
    name = item.name
    value = getattr(model, name)
    if not isinstance(value, np.ndarray):
      continue
    value = np.ascontiguousarray(getattr(model, name))
    digest.update(name.encode())
    digest.update(value.dtype.str.encode())
    digest.update(value.tobytes())
  return digest.hexdigest()


def _perturb_quaternions(base, rotation_vector):
  result = np.empty_like(base)
  for index, (quat, vector) in enumerate(zip(base, rotation_vector)):
    angle = np.linalg.norm(vector)
    if angle == 0:
      result[index] = quat / np.linalg.norm(quat)
      continue
    delta = np.r_[np.cos(angle / 2), vector * (np.sin(angle / 2) / angle)]
    w, x, y, z = quat / np.linalg.norm(quat)
    a, b, c, d = delta
    result[index] = [
        w * a - x * b - y * c - z * d,
        w * b + x * a + y * d - z * c,
        w * c - x * d + y * a + z * b,
        w * d + x * c - y * b + z * a,
    ]
  return result


def _clone_model(model):
  """Copy an MjModel with MuJoCo's supported Python copy semantics."""
  return copy(model)


def _compile_source(source):
  if isinstance(source, mujoco.MjModel):
    return _clone_model(source)
  if isinstance(source, bytes):
    return mujoco.MjModel.from_xml_string(source.decode("utf-8"))
  if isinstance(source, Path) or (
      isinstance(source, str) and "<" not in source
  ):
    return mujoco.MjModel.from_xml_path(str(source))
  return mujoco.MjModel.from_xml_string(str(source))


class ModelLifecycle:
  """Owns constants and recomputes derived fields transactionally on CPU."""

  def __init__(self, source):
    if mujoco.__version__ != "3.10.0":
      raise RuntimeError(f"requires MuJoCo 3.10.0; found {mujoco.__version__}")
    self._model = _compile_source(source)
    if self._model.nmocap:
      raise ValueError(
          "mocap inputs are unsupported by the current kinematics stage"
      )
    self.descriptor = load_model(self._model)
    self.generation = 0

  def recompute_body_masses(self, body_ids, masses):
    """Set body masses, run `mj_setConst`, and commit only a valid candidate."""
    raw_ids = np.asarray(body_ids)
    if raw_ids.dtype.kind not in "iu":
      raise ValueError("body_ids must contain integers")
    ids = raw_ids.astype(np.int64, copy=False)
    values = np.asarray(masses, dtype=np.float64)
    if ids.ndim != 1 or values.shape != ids.shape or ids.size == 0:
      raise ValueError("body_ids and masses must be matching nonempty vectors")
    if np.unique(ids).size != ids.size:
      raise ValueError("body_ids must be unique")
    if np.any(ids <= 0) or np.any(ids >= self.descriptor.nbody):
      raise ValueError("body_ids must refer to non-world bodies")
    base_mass = self.descriptor.body_mass[ids]
    if (
        not np.all(np.isfinite(values))
        or np.any((base_mass > 0) & (values <= 0))
        or np.any((base_mass == 0) & (values != 0))
    ):
      raise ValueError(
          "masses must preserve zero-mass fixed bodies and keep other masses positive"
      )
    if np.array_equal(self.descriptor.body_mass[ids], values):
      return False

    # Changes stay private until recomputation and lowering both succeed.
    candidate = _clone_model(self._model)
    candidate.body_mass[ids] = values
    mujoco.mj_setConst(candidate, mujoco.MjData(candidate))
    descriptor = load_model(candidate)
    self._model = candidate
    self.descriptor = descriptor
    self.generation += 1
    return True


@dataclass(frozen=True)
class BatchSnapshot:
  """Immutable qpos checkpoint for a kinematics batch."""

  nq: int
  batch_size: int
  model_fingerprint: str
  qpos: np.ndarray
  schema_version: int = 1

  def __post_init__(self):
    value = np.asarray(self.qpos)
    if value.dtype.kind not in "fiu":
      raise ValueError("snapshot qpos must be numeric")
    frozen = np.frombuffer(
        np.ascontiguousarray(value).tobytes(), dtype=value.dtype
    ).reshape(value.shape)
    object.__setattr__(self, "qpos", frozen)


class KinematicsBatchState:
  """Batched qpos with per-row generation and FK cache invalidation."""

  def __init__(self, model: ModelDescriptor, batch_size: int):
    if (
        isinstance(batch_size, (bool, np.bool_))
        or not isinstance(batch_size, (int, np.integer))
        or batch_size <= 0
    ):
      raise ValueError("batch_size must be a positive integer")
    self.model = snapshot_descriptor(model)
    self._fingerprint = _fingerprint(self.model)
    self._qpos = np.broadcast_to(
        self.model.qpos0, (batch_size, self.model.nq)
    ).copy()
    self._row_generation = np.zeros(batch_size, dtype=np.int64)
    self._cache = [None] * batch_size
    self._cache_generation = np.full(batch_size, -1, dtype=np.int64)

  @property
  def qpos(self):
    """Read-only detached state view."""
    result = self._qpos.copy()
    result.setflags(write=False)
    return result

  @property
  def row_generation(self):
    """Read-only detached per-row cache generations."""
    result = self._row_generation.copy()
    result.setflags(write=False)
    return result

  def _ids(self, env_ids):
    raw = np.asarray(env_ids)
    if raw.dtype.kind not in "iu":
      raise ValueError("env_ids must contain integers")
    ids = raw.astype(np.int64, copy=False)
    if ids.ndim != 1 or np.unique(ids).size != ids.size:
      raise ValueError("env_ids must be a vector of unique environment indices")
    if np.any(ids < 0) or np.any(ids >= self._qpos.shape[0]):
      raise ValueError("env_ids are out of range")
    return ids

  def set_qpos(self, env_ids, values):
    """Update selected rows atomically; unchanged rows keep cached poses."""
    ids = self._ids(env_ids)
    values = np.asarray(values, dtype=np.float64)
    if values.shape != (ids.size, self.model.nq) or not np.all(
        np.isfinite(values)
    ):
      raise ValueError("qpos values must be finite and match selected rows")
    changed = []
    for index, env_id in enumerate(ids):
      if np.array_equal(values[index], self._qpos[env_id]):
        continue
      self.model.forward_kinematics(values[index])
      changed.append((int(env_id), values[index].copy()))
    for env_id, value in changed:
      self._qpos[env_id] = value
      self._row_generation[env_id] += 1
      self._cache[env_id] = None
      self._cache_generation[env_id] = -1
    return tuple(env_id for env_id, _ in changed)

  def poses(self, env_id):
    """Return cached CPU FK poses for one row, copying arrays for ownership."""
    ids = self._ids([env_id])
    index = int(ids[0])
    if self._cache_generation[index] != self._row_generation[index]:
      self._cache[index] = self.model.forward_kinematics(self._qpos[index])
      self._cache_generation[index] = self._row_generation[index]
    return {name: value.copy() for name, value in self._cache[index].items()}

  def snapshot(self):
    """Capture a detached immutable checkpoint."""
    raw = self.qpos.copy()
    frozen = np.frombuffer(raw.tobytes(), dtype=raw.dtype).reshape(raw.shape)
    return BatchSnapshot(
        self.model.nq, self._qpos.shape[0], self._fingerprint, frozen
    )

  def restore(self, snapshot):
    """Restore all rows after validating them; invalidate every cached pose."""
    if (
        snapshot.schema_version != 1
        or snapshot.nq != self.model.nq
        or snapshot.batch_size != self._qpos.shape[0]
        or snapshot.model_fingerprint != self._fingerprint
    ):
      raise ValueError(
          "snapshot schema, dimensions, or model fingerprint do not match"
      )
    values = np.asarray(snapshot.qpos, dtype=np.float64)
    if values.shape != self._qpos.shape:
      raise ValueError("snapshot qpos shape is invalid")
    for value in values:
      self.model.forward_kinematics(value)
    self._qpos[:] = values
    self._row_generation += 1
    self._cache = [None] * self._qpos.shape[0]
    self._cache_generation[:] = -1

  def randomize(self, env_ids, seed, scale=0.1):
    """Randomize valid joint coordinates in selected rows deterministically."""
    if not np.isfinite(scale) or scale < 0:
      raise ValueError("scale must be finite and nonnegative")
    ids = self._ids(env_ids)
    rng = np.random.default_rng(seed)
    values = np.broadcast_to(self.model.qpos0, (ids.size, self.model.nq)).copy()
    for joint, typ in enumerate(self.model.jnt_type):
      qa = int(self.model.jnt_qposadr[joint])
      typ = int(typ)
      if typ == int(mujoco.mjtJoint.mjJNT_FREE):
        values[:, qa : qa + 3] += rng.normal(0, scale, (ids.size, 3))
        values[:, qa + 3 : qa + 7] = _perturb_quaternions(
            values[:, qa + 3 : qa + 7], rng.normal(size=(ids.size, 3)) * scale
        )
      elif typ == int(mujoco.mjtJoint.mjJNT_BALL):
        values[:, qa : qa + 4] = _perturb_quaternions(
            values[:, qa : qa + 4], rng.normal(size=(ids.size, 3)) * scale
        )
      else:
        values[:, qa] += rng.normal(0, scale, ids.size)
    return self.set_qpos(ids, values)


@dataclass(frozen=True)
class ConstantsSnapshot:
  """Immutable batched model-parameter checkpoint."""

  batch_size: int
  nbody: int
  model_fingerprint: str
  body_mass: np.ndarray
  schema_version: int = 1

  def __post_init__(self):
    value = np.asarray(self.body_mass)
    if value.dtype.kind not in "fiu":
      raise ValueError("snapshot body_mass must be numeric")
    frozen = np.frombuffer(
        np.ascontiguousarray(value).tobytes(), dtype=value.dtype
    ).reshape(value.shape)
    object.__setattr__(self, "body_mass", frozen)


class BatchedConstants:
  """Per-environment model constants with transactional CPU recomputation."""

  def __init__(self, source, batch_size):
    if mujoco.__version__ != "3.10.0":
      raise RuntimeError(f"requires MuJoCo 3.10.0; found {mujoco.__version__}")
    if (
        isinstance(batch_size, (bool, np.bool_))
        or not isinstance(batch_size, (int, np.integer))
        or batch_size <= 0
    ):
      raise ValueError("batch_size must be a positive integer")
    self._base_model = _compile_source(source)
    if self._base_model.nmocap:
      raise ValueError(
          "mocap inputs are unsupported by the current kinematics stage"
      )
    self.descriptor = load_model(self._base_model)
    self._fingerprint = _fingerprint(self._base_model)
    self._batch_size = int(batch_size)
    self._body_mass = np.broadcast_to(
        self.descriptor.body_mass, (self._batch_size, self.descriptor.nbody)
    ).copy()
    self._body_invweight0 = np.empty(
        (self._batch_size,) + self._base_model.body_invweight0.shape
    )
    self._row_generation = np.zeros(self._batch_size, dtype=np.int64)
    self.generation = 0
    for env_id in range(self._batch_size):
      self._body_invweight0[env_id] = self._derive(self._body_mass[env_id])

  @property
  def batch_size(self):
    return self._batch_size

  @property
  def body_mass(self):
    result = self._body_mass.copy()
    result.setflags(write=False)
    return result

  @property
  def body_invweight0(self):
    result = self._body_invweight0.copy()
    result.setflags(write=False)
    return result

  @property
  def row_generation(self):
    result = self._row_generation.copy()
    result.setflags(write=False)
    return result

  def _env_ids(self, env_ids):
    raw = np.asarray(env_ids)
    if raw.dtype.kind not in "iu":
      raise ValueError("env_ids must contain integers")
    ids = raw.astype(np.int64, copy=False)
    if ids.ndim != 1 or ids.size == 0 or np.unique(ids).size != ids.size:
      raise ValueError("env_ids must be a nonempty vector of unique indices")
    if np.any(ids < 0) or np.any(ids >= self._batch_size):
      raise ValueError("env_ids are out of range")
    return ids

  def _derive(self, masses):
    candidate = _clone_model(self._base_model)
    candidate.body_mass[:] = masses
    mujoco.mj_setConst(candidate, mujoco.MjData(candidate))
    result = np.array(candidate.body_invweight0, copy=True)
    if not np.all(np.isfinite(result)):
      raise ValueError("mj_setConst produced nonfinite body_invweight0")
    return result

  def _commit(self, env_ids, candidates, force=False):
    changed = [
        int(env_id)
        for env_id in env_ids
        if force
        or not np.array_equal(candidates[env_id], self._body_mass[env_id])
    ]
    if not changed:
      return ()
    recomputed = {}
    # Compute every selected derived row before mutating published state.
    for env_id in changed:
      recomputed[env_id] = self._derive(candidates[env_id])
    for env_id in changed:
      self._body_mass[env_id] = candidates[env_id]
      self._body_invweight0[env_id] = recomputed[env_id]
      self._row_generation[env_id] += 1
    self.generation += 1
    return tuple(changed)

  def recompute_constants(self, parameters, env_ids):
    """Recompute rows from complete per-world `body_mass` parameter vectors."""
    ids = self._env_ids(env_ids)
    if not isinstance(parameters, dict) or set(parameters) != {"body_mass"}:
      raise ValueError("parameters must contain only a body_mass array")
    values = np.asarray(parameters["body_mass"], dtype=np.float64)
    shape = (ids.size, self.descriptor.nbody)
    if values.shape != shape or not np.all(np.isfinite(values)):
      raise ValueError(f"body_mass must be finite with shape {shape}")
    if not np.allclose(
        values[:, 0], self._base_model.body_mass[0], rtol=0, atol=0
    ):
      raise ValueError("world body mass is immutable")
    base_mass = self._base_model.body_mass[None, 1:]
    if np.any((base_mass > 0) & (values[:, 1:] <= 0)) or np.any(
        (base_mass == 0) & (values[:, 1:] != 0)
    ):
      raise ValueError("mass parameters must preserve massless fixed bodies")
    candidate = self._body_mass.copy()
    candidate[ids] = values
    return self._commit(ids, candidate)

  def set_body_masses(self, env_ids, body_ids, masses):
    """Update selected bodies in selected worlds, preserving other rows."""
    ids = self._env_ids(env_ids)
    raw_body_ids = np.asarray(body_ids)
    if raw_body_ids.dtype.kind not in "iu":
      raise ValueError("body_ids must contain integers")
    body_ids = raw_body_ids.astype(np.int64, copy=False)
    if body_ids.ndim != 1 or body_ids.size == 0:
      raise ValueError("body_ids must be a nonempty vector")
    if np.unique(body_ids).size != body_ids.size:
      raise ValueError("body_ids must be unique")
    if np.any(body_ids <= 0) or np.any(body_ids >= self.descriptor.nbody):
      raise ValueError("body_ids must refer to non-world bodies")
    values = np.asarray(masses, dtype=np.float64)
    if values.shape != (ids.size, body_ids.size) or not np.all(
        np.isfinite(values)
    ):
      raise ValueError(
          "masses must be finite and match selected worlds and bodies"
      )
    base_mass = self._base_model.body_mass[body_ids]
    if np.any((base_mass[None, :] > 0) & (values <= 0)) or np.any(
        (base_mass[None, :] == 0) & (values != 0)
    ):
      raise ValueError("mass values must preserve massless fixed bodies")
    candidate = self._body_mass.copy()
    candidate[np.ix_(ids, body_ids)] = values
    return self._commit(ids, candidate)

  def snapshot(self):
    raw = self._body_mass.copy()
    frozen = np.frombuffer(raw.tobytes(), dtype=raw.dtype).reshape(raw.shape)
    return ConstantsSnapshot(
        self._batch_size, self.descriptor.nbody, self._fingerprint, frozen, 1
    )

  def restore(self, snapshot):
    if (
        snapshot.schema_version != 1
        or snapshot.batch_size != self._batch_size
        or snapshot.nbody != self.descriptor.nbody
        or snapshot.model_fingerprint != self._fingerprint
    ):
      raise ValueError("snapshot schema or model fingerprint does not match")
    values = np.asarray(snapshot.body_mass, dtype=np.float64)
    if values.shape != self._body_mass.shape or not np.all(np.isfinite(values)):
      raise ValueError("snapshot body_mass is invalid")
    if not np.allclose(
        values[:, 0], self._base_model.body_mass[0], rtol=0, atol=0
    ):
      raise ValueError("snapshot changes world body mass")
    base_mass = self._base_model.body_mass[None, 1:]
    if np.any((base_mass > 0) & (values[:, 1:] <= 0)) or np.any(
        (base_mass == 0) & (values[:, 1:] != 0)
    ):
      raise ValueError("snapshot changes massless fixed bodies")
    # Force recomputation even for equal parameters so derived caches are fresh.
    candidates = values.copy()
    self._commit(np.arange(self._batch_size), candidates, force=True)

  def randomize(self, env_ids, seed, scale=0.1):
    """Apply seeded log-normal mass factors; scale zero is a true no-op."""
    if not np.isfinite(scale) or scale < 0:
      raise ValueError("scale must be finite and nonnegative")
    ids = self._env_ids(env_ids)
    body_ids = np.flatnonzero(self._base_model.body_mass[1:] > 0) + 1
    if not body_ids.size or scale == 0:
      return ()
    rng = np.random.default_rng(seed)
    values = self._body_mass[np.ix_(ids, body_ids)] * np.exp(
        rng.normal(0, scale, (ids.size, body_ids.size))
    )
    return self.set_body_masses(ids, body_ids, values)
