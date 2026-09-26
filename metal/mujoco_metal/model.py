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

"""CPU-only loading and support checks for the first experimental profile."""

import hashlib

import mujoco
import numpy as np

from mujoco_metal import _model

PROFILE = "microduck-flat-v1"
MUJOCO_VERSION = "3.10.0"


def check_version():
  """Reject unqualified MuJoCo semantics before model or GPU allocation."""
  if mujoco.__version__ != MUJOCO_VERSION:
    raise RuntimeError(
        f"This experimental profile requires mujoco=={MUJOCO_VERSION}; "
        f"found {mujoco.__version__}. Newer versions need requalification."
    )


def load_canonical():
  """Load the bounded profile with only the supported foot/ground collisions."""
  check_version()
  canonical = _model.load_canonical_model()
  model = canonical.model
  model.geom_contype[:] = 0
  model.geom_conaffinity[:] = 0
  for geom in (
      canonical.terrain_geom_id,
      canonical.left_foot_geom_id,
      canonical.right_foot_geom_id,
  ):
    model.geom_contype[geom] = 1 if geom == canonical.terrain_geom_id else 2
    model.geom_conaffinity[geom] = 2 if geom == canonical.terrain_geom_id else 1
  model.opt.solver = mujoco.mjtSolver.mjSOL_PGS
  model.opt.iterations = 100
  model.opt.tolerance = 1e-5
  return canonical


def load_model():
  """Return an independent CPU model for the supported foot-ground profile.

  Self-collision and non-foot ground collision are disabled explicitly. This
  model is a physics example, not the full downstream walking task.
  """
  return load_canonical().model


def model_fingerprint(model):
  """Hash compiled arrays/options to reject silent unsupported modifications."""
  digest = hashlib.sha256()
  for prefix, obj in (("model", model), ("option", model.opt)):
    for name in sorted(dir(obj)):
      if name.startswith("_"):
        continue
      value = getattr(obj, name)
      if isinstance(value, np.ndarray):
        digest.update(f"{prefix}.{name}:{value.dtype}:{value.shape}".encode())
        digest.update(value.tobytes())
      elif isinstance(value, (int, float, bool, np.number)):
        digest.update(f"{prefix}.{name}:{value!r}".encode())
  return digest.hexdigest()


def validate_model(model, reference=None):
  """Require the exact supported profile; model generalization is deferred."""
  check_version()
  if not isinstance(model, mujoco.MjModel):
    raise TypeError("model must be a mujoco.MjModel")
  reference = load_model() if reference is None else reference
  if model_fingerprint(model) != model_fingerprint(reference):
    raise ValueError(
        "Unsupported model or modified parameters. This port currently accepts "
        "only mujoco_metal.load_model() without changes (microduck-flat-v1)."
    )
