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

"""CPU-only package preflight and model inspection."""

import argparse
from dataclasses import asdict
from enum import Enum
import hashlib
import json
from pathlib import Path

import mujoco

from mujoco_metal import __version__
from mujoco_metal.model import load_model
from mujoco_metal.registry import FEATURES
from mujoco_metal.registry import INVENTORY_COMPLETE
from mujoco_metal.registry import TARGET_MUJOCO_VERSION


def _jsonable(value):
  if isinstance(value, Enum):
    return value.value
  if isinstance(value, dict):
    return {key: _jsonable(item) for key, item in value.items()}
  if isinstance(value, (list, tuple)):
    return [_jsonable(item) for item in value]
  return value


def preflight(model_path=None, include_inventory=False):
  package = Path(__file__).resolve().parent
  shader = package / "shaders" / "kinematics.metal"
  shader_paths = {
      "kinematics": shader,
      "smooth_mass": package / "shaders" / "smooth_mass.metal",
      "smooth_bias": package / "shaders" / "smooth_bias.metal",
      "smooth_solve": package / "shaders" / "smooth_solve.metal",
      "integration": package / "shaders" / "integration.metal",
  }
  result = {
      "package_version": __version__,
      "package_path": str(package),
      "target_mujoco_version": TARGET_MUJOCO_VERSION,
      "actual_mujoco_version": mujoco.__version__,
      "shader_path": str(shader),
      "shader_sha256": hashlib.sha256(shader.read_bytes()).hexdigest(),
      "shaders": {
          name: {
              "path": str(path),
              "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
          }
          for name, path in shader_paths.items()
      },
      "inventory_complete": INVENTORY_COMPLETE,
      "gpu_qualified": False,
      "gpu_qualification_scope": "complete physics backend; per-stage narrow results are listed in feature inventory",
      "stages": {
          "model_inspection": "CPU",
          "kinematics": "CPU oracle; Metal narrowly GPU-qualified on M1 fixtures",
          "dynamics": "CPU smooth M/bias oracle; Metal narrowly GPU-qualified on M1 fixtures",
          "acceleration_solve": "native dense SPD solve; narrowly GPU-qualified on M1 synthetic systems",
          "device_state": "persistent MPS state with host reset/checkpoint lifecycle; pipeline qualification separate",
          "integration": "native semi-implicit Euler; narrowly GPU-qualified against MuJoCo 3.10 on M1 fixtures",
          "contact_free_euler_v1": "narrowly GPU-qualified on M1 fixtures; bounded profile only",
          "full_stepping": "unsupported beyond the bounded contact-free Euler profile",
          "collision": "unsupported",
          "constraints": "unsupported",
          "sensors": "unsupported",
          "rendering": "unsupported",
      },
  }
  if model_path:
    model = load_model(Path(model_path))
    result["model"] = {
        name: getattr(model, name)
        for name in (
            "nq",
            "nv",
            "nu",
            "nbody",
            "njnt",
            "ngeom",
            "nsite",
            "ntendon",
            "nmocap",
        )
    }
  if include_inventory:
    result["features"] = [_jsonable(asdict(row)) for row in FEATURES]
  return result


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  subparsers = parser.add_subparsers(dest="command", required=True)
  preflight_parser = subparsers.add_parser("preflight")
  preflight_parser.add_argument("--model", type=Path)
  preflight_parser.add_argument("--json", action="store_true", dest="as_json")
  preflight_parser.add_argument("--inventory", action="store_true")
  args = parser.parse_args()
  result = preflight(args.model, args.inventory)
  if args.as_json:
    print(json.dumps(result, indent=2, sort_keys=True))
  else:
    for key, value in result.items():
      if key != "features":
        print(f"{key}: {value}")
    if args.inventory:
      print(f"features: {len(result['features'])} inventory rows")


if __name__ == "__main__":
  main()
