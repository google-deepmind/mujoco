# Copyright 2026 The MuJoCo Metal contributors
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy at https://www.apache.org/licenses/LICENSE-2.0
# Unless required by law or agreed to in writing, software distributed under the
# License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS
# OF ANY KIND, either express or implied. See the License for the specific
# language governing permissions and limitations under the License.
"""Separate-process CPU8/Metal timing for the contact-free demo pendulum.

CPU rollout writes full float64 trajectories; Metal retains float32 device state.
This intentionally measures those APIs, not identical output or precision costs.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import statistics
import time

import mujoco
from mujoco import rollout
import numpy as np


def positive_int(value):
  value = int(value)
  if value <= 0:
    raise argparse.ArgumentTypeError("must be positive")
  return value


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--backend", choices=("cpu8", "metal"), required=True)
  parser.add_argument("--batch", type=positive_int, required=True)
  parser.add_argument("--steps", type=positive_int, default=200)
  parser.add_argument("--trials", type=positive_int, default=3)
  parser.add_argument("--output", type=Path, required=True)
  args = parser.parse_args()
  model_path = Path(__file__).resolve().parents[1] / "examples/pendulum.xml"
  model = mujoco.MjModel.from_xml_path(str(model_path))
  qpos = np.tile(model.qpos0, (args.batch, 1))
  qvel = np.zeros((args.batch, model.nv))
  qvel[:, 0] = 10
  record = {
      "backend": args.backend,
      "batch": args.batch,
      "steps": args.steps,
      "trials": args.trials,
      "mujoco": mujoco.__version__,
      "numpy": np.__version__,
      "platform": platform.platform(),
      "model_sha256": hashlib.sha256(model_path.read_bytes()).hexdigest(),
  }
  times = []
  if args.backend == "cpu8":
    initial = np.concatenate((np.zeros((args.batch, 1)), qpos, qvel), axis=1)
    shape = (args.batch, args.steps, initial.shape[1])
    output_bytes = int(np.prod(shape)) * 8
    if output_bytes > 8 * 1024**3:
      parser.error("CPU trajectory output would exceed the 8 GiB guard")
    state = np.empty(shape)
    sensors = np.empty((args.batch, args.steps, model.nsensordata))
    with rollout.Rollout(nthread=8) as runner:
      data = [mujoco.MjData(model) for _ in range(8)]

      def run():
        runner.rollout(
            [model] * args.batch,
            data,
            initial,
            nstep=args.steps,
            state=state,
            sensordata=sensors,
        )

      run()
      for _ in range(args.trials):
        start = time.perf_counter()
        run()
        times.append(time.perf_counter() - start)
    final_qpos = state[:, -1, 1 : 1 + model.nq]
    final_qvel = state[:, -1, 1 + model.nq :]
    record.update(cpu_threads=8, trajectory_buffer_bytes=output_bytes)
  else:
    if os.environ.get("PYTORCH_ENABLE_MPS_FALLBACK") != "0":
      parser.error("set PYTORCH_ENABLE_MPS_FALLBACK=0 before launch")
    # Keep CPU measurement independent of Torch/MPS initialization.
    import torch  # pylint: disable=g-import-not-at-top
    from mujoco_metal import MetalSimulation  # pylint: disable=g-import-not-at-top

    sim = MetalSimulation(model, args.batch, qpos=qpos, qvel=qvel)
    sim.step(50)
    torch.mps.synchronize()
    for _ in range(args.trials):
      sim.state.reset(qpos=qpos, qvel=qvel)
      torch.mps.synchronize()
      start = time.perf_counter()
      sim.step(args.steps)
      torch.mps.synchronize()
      times.append(time.perf_counter() - start)
    snapshot = sim.state.snapshot()
    if np.any(snapshot.status != 0):
      raise RuntimeError("native solver/integration failure")
    final_qpos, final_qvel = snapshot.qpos, snapshot.qvel
    record.update(
        torch=torch.__version__,
        warmup_steps=50,
        mps_allocated_bytes=torch.mps.current_allocated_memory(),
        mps_driver_bytes=torch.mps.driver_allocated_memory(),
    )
  reference = mujoco.MjData(model)
  reference.qvel[0] = 10
  for _ in range(args.steps):
    mujoco.mj_step(model, reference)
  np.testing.assert_allclose(final_qpos - reference.qpos, 0, atol=1e-4, rtol=0)
  np.testing.assert_allclose(final_qvel - reference.qvel, 0, atol=1e-3, rtol=0)
  record.update(
      wall_seconds=times,
      median_wall_seconds=statistics.median(times),
      world_steps_per_second=args.batch * args.steps / statistics.median(times),
      max_qpos_error=float(np.max(np.abs(final_qpos - reference.qpos))),
      max_qvel_error=float(np.max(np.abs(final_qvel - reference.qvel))),
  )
  args.output.parent.mkdir(parents=True, exist_ok=True)
  args.output.write_text(json.dumps(record, indent=2) + "\n")
  print(json.dumps(record, indent=2))


if __name__ == "__main__":
  main()
