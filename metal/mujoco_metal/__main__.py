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

"""Explicit CPU preflight and GPU smoke commands; no implicit benchmark."""

import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path


def preflight():
  from mujoco_metal.model import load_model
  from mujoco_metal.model import model_fingerprint
  from mujoco_metal.model import PROFILE

  model = load_model()
  root = Path(__file__).resolve().parent
  shader = root / "shaders" / "physics_slice.metal"
  return {
      "profile": PROFILE,
      "model_fingerprint": model_fingerprint(model),
      "module_path": str(root),
      "shader_path": str(shader),
      "shader_sha256": hashlib.sha256(shader.read_bytes()).hexdigest(),
      "mujoco": importlib.metadata.version("mujoco"),
      "nq": model.nq,
      "nv": model.nv,
      "nu": model.nu,
      "gpu_executed": False,
  }


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("command", choices=("preflight", "smoke"))
  parser.add_argument("--batch-size", type=int, default=4)
  parser.add_argument("--steps", type=int, default=10)
  args = parser.parse_args()
  report = preflight()
  if args.command == "smoke":
    from mujoco_metal.simulation import positive_int
    from mujoco_metal.simulation import Simulation

    positive_int(args.steps, "steps")
    positive_int(args.batch_size, "batch_size")
    import torch

    sim = Simulation(batch_size=args.batch_size)
    ctrl = torch.zeros((args.batch_size, 14), device="mps", dtype=torch.float32)
    unconverged = 0
    for _ in range(args.steps):
      output = sim.step(ctrl)
      unconverged += int((output.solver_status == 1).sum().item())
    saved = sim.state_dict()
    sim.reset()
    sim.load_state_dict(saved)
    torch.mps.synchronize()
    report.update(
        gpu_executed=True,
        steps=args.steps,
        batch_size=args.batch_size,
        unconverged_world_steps=unconverged,
        simulation_time=float(saved["time"][0]),
    )
  print(json.dumps(report, indent=2))


if __name__ == "__main__":
  main()
