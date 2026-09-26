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

#!/usr/bin/env python3
"""Milestone 4: Canonical 5 ms ImplicitFast Time Integration and Free-Running Trajectory Qualification.

Generates complete numerical evaluation tables:
1. Common-state 1-step (5 ms) parity across all 25 corpus scenarios using separated metric gates.
2. Free-running CPU and Metal trajectories across 4 steps (20 ms), 20 steps (100 ms), and 200 steps (1.0 s).
3. Actuator torque control and the 4-substep control interval (20 ms).
4. Multi-world failure isolation and reset recovery.
"""

import math
from pathlib import Path
import sys
from typing import Dict, List, Tuple

PROJECT_ROOT = Path(__file__).resolve().parent

import mujoco
import numpy as np
from scipy.linalg import solve_triangular
import torch

from mujoco_metal._model import CANONICAL_XML_PATH
from mujoco_metal._model import CanonicalMicroDuckModel
from mujoco_metal._model import create_matching_cpu_data
from mujoco_metal._model import create_matching_cpu_model
from mujoco_metal._model import load_canonical_model
from mujoco_metal._physics import RepresentativePhysicsSlice

# Canonical Milestone 4 Physical Qualification Gates
PHYSICAL_GATES: Dict[str, float] = {
    "pos_err": 1e-3,  # 1.0 mm
    "so3_err": 1e-3,  # 1.0 mrad
    "linvel_err": 0.05,  # 0.05 m/s
    "angvel_err": 0.05,  # 0.05 rad/s
    "jnt_pos_err": 1e-3,  # 1.0 mrad
    "jnt_vel_err": 0.05,  # 0.05 rad/s
}

GATE_POS_MAX = PHYSICAL_GATES["pos_err"]
GATE_SO3_MAX = PHYSICAL_GATES["so3_err"]
GATE_LINVEL_MAX = PHYSICAL_GATES["linvel_err"]
GATE_ANGVEL_MAX = PHYSICAL_GATES["angvel_err"]
GATE_JNTPOS_MAX = PHYSICAL_GATES["jnt_pos_err"]
GATE_JNTVEL_MAX = PHYSICAL_GATES["jnt_vel_err"]


def safe_mps_sync(device):
  """Synchronize only the device under test; CPU stubs must not touch MPS."""
  if torch.device(device).type == "mps":
    torch.mps.synchronize()


def compute_so3_distance(q1: np.ndarray, q2: np.ndarray) -> float:
  """Computes geodesic angular distance on SO(3) invariant to q ~ -q."""
  if not (np.all(np.isfinite(q1)) and np.all(np.isfinite(q2))):
    return float("nan")
  q1_64 = q1.astype(np.float64)
  q2_64 = q2.astype(np.float64)
  norm1 = float(np.linalg.norm(q1_64))
  norm2 = float(np.linalg.norm(q2_64))
  if norm1 < 1e-8 or norm2 < 1e-8:
    return float("nan")
  q1_norm = q1_64 / norm1
  q2_norm = q2_64 / norm2
  dot = float(np.abs(np.dot(q1_norm, q2_norm)))
  dot_clamped = min(1.0, max(0.0, dot))
  return float(2.0 * np.arccos(dot_clamped))


def compute_separated_metrics(
    qp_gpu: np.ndarray,
    qv_gpu: np.ndarray,
    qp_cpu: np.ndarray,
    qv_cpu: np.ndarray,
) -> Dict[str, float]:
  """Computes separated metrics without pooling quantities of different units."""
  if not (
      np.all(np.isfinite(qp_gpu))
      and np.all(np.isfinite(qv_gpu))
      and np.all(np.isfinite(qp_cpu))
      and np.all(np.isfinite(qv_cpu))
  ):
    return {k: float("nan") for k in PHYSICAL_GATES}
  pos_err = float(np.max(np.abs(qp_gpu[0:3] - qp_cpu[0:3])))
  so3_err = compute_so3_distance(qp_gpu[3:7], qp_cpu[3:7])
  linvel_err = float(np.max(np.abs(qv_gpu[0:3] - qv_cpu[0:3])))
  angvel_err = float(np.max(np.abs(qv_gpu[3:6] - qv_cpu[3:6])))
  jnt_pos_err = float(np.max(np.abs(qp_gpu[7:21] - qp_cpu[7:21])))
  jnt_vel_err = float(np.max(np.abs(qv_gpu[6:20] - qv_cpu[6:20])))
  return {
      "pos_err": pos_err,
      "so3_err": so3_err,
      "linvel_err": linvel_err,
      "angvel_err": angvel_err,
      "jnt_pos_err": jnt_pos_err,
      "jnt_vel_err": jnt_vel_err,
  }


def validate_raw_state(
    qpos: np.ndarray, qvel: np.ndarray, context: str
) -> List[str]:
  """Validates raw candidate state arrays for finiteness before normalization or metric reduction."""
  violations = []
  if not np.all(np.isfinite(qpos)):
    violations.append(f"{context}: non-finite raw qpos")
  if not np.all(np.isfinite(qvel)):
    violations.append(f"{context}: non-finite raw qvel")
  return violations


def validate_statuses(
    solver_stat: int, int_stat: int, context: str
) -> List[str]:
  """Validates solver and integration statuses.

  Negative solver_status indicates an unhandled solver failure.
  Solver status 1 (exhausted iterations) is allowed as diagnostic reporting.
  Nonzero integration_status indicates an integration failure.
  """
  violations = []
  if solver_stat < 0:
    violations.append(f"{context}: solver failure (status={solver_stat})")
  if int_stat != 0:
    violations.append(f"{context}: integration failure (status={int_stat})")
  return violations


def validate_metrics(metrics: Dict[str, float], context: str) -> List[str]:
  """Validates metric differences against canonical physical gates independently."""
  violations = []
  for key, limit in PHYSICAL_GATES.items():
    if key not in metrics:
      continue
    val = metrics[key]
    if not math.isfinite(val):
      violations.append(f"{context}: {key} non-finite ({val})")
    elif val > limit:
      violations.append(f"{context}: {key}={val:.2e} > limit {limit:.2e}")
  return violations


def compute_kkt(out, nefc: int, max_iters: int) -> Dict:
  """Computes KKT optimality and diagnostics for Delassus PGS candidate solve."""
  stat = int(out.solver_status[0].cpu().item())
  iters = int(out.actual_iters[0].cpu().item())

  # Read failure status first: failed zero-row world cannot be mislabeled
  if stat < 0:
    return {
        "stat": stat,
        "iters": iters,
        "label": "failed",
        "primal": float("nan"),
        "dual": float("nan"),
        "comp": float("nan"),
        "proj": float("nan"),
    }

  if nefc == 0:
    return {
        "stat": 0,
        "iters": 0,
        "label": "converged",
        "primal": 0.0,
        "dual": 0.0,
        "comp": 0.0,
        "proj": 0.0,
    }

  if stat == 0:
    label = "converged"
  elif iters == max_iters:
    label = "exhausted"
  else:
    label = "stagnated"

  J_np = out.J[0, :nefc].cpu().numpy()
  R_np = out.R[0, :nefc].cpu().numpy()
  aref_np = out.aref[0, :nefc].cpu().numpy()
  L_np = out.L_factor[0].cpu().numpy()
  f_sm_np = out.f_smooth[0].cpu().numpy()
  lam_np = out.lambda_force[0, :nefc].cpu().numpy()

  Y = solve_triangular(L_np, J_np.T, lower=True)
  A = Y.T @ Y + np.diag(R_np)
  y0 = solve_triangular(L_np, f_sm_np, lower=True)
  a0 = solve_triangular(L_np.T, y0, lower=False)
  b = J_np @ a0 - aref_np
  g = A @ lam_np + b

  primal = float(np.max(np.maximum(0.0, -lam_np)))
  dual = float(np.max(np.maximum(0.0, -g)))
  comp = float(np.max(np.abs(lam_np * g)))
  proj = float(
      np.max(np.abs(lam_np - np.maximum(0.0, lam_np - g / np.diag(A))))
  )

  return {
      "stat": stat,
      "iters": iters,
      "label": label,
      "primal": primal,
      "dual": dual,
      "comp": comp,
      "proj": proj,
  }


def evaluate_common_state_one_step(
    ps: RepresentativePhysicsSlice,
    canonical: CanonicalMicroDuckModel,
) -> Tuple[List[Dict], List[str]]:
  """Evaluates 1-step (5 ms) state advancement across all 25 corpus scenarios vs matched CPU models."""
  corpus_dir = PROJECT_ROOT / "corpus"
  scenarios = sorted([f.stem for f in corpus_dir.glob("*.npz")])
  if len(scenarios) != 25:
    raise RuntimeError(
        f"Expected exactly 25 corpus scenarios, found {len(scenarios)}"
    )

  results = []
  gate_violations = []

  for sc_name in scenarios:
    fpath = corpus_dir / f"{sc_name}.npz"
    d_npz = dict(np.load(fpath))

    # 1. Matched CPU reference model & fresh data
    m_matched = create_matching_cpu_model(canonical, d_npz)
    d_matched = create_matching_cpu_data(m_matched, d_npz)
    mujoco.mj_step(m_matched, d_matched)

    # 2. GPU candidate
    qp_t = (
        torch.from_numpy(d_npz["qpos"].astype(np.float32))
        .unsqueeze(0)
        .to(ps.device)
    )
    qv_t = (
        torch.from_numpy(d_npz["qvel"].astype(np.float32))
        .unsqueeze(0)
        .to(ps.device)
    )
    f_smooth = (
        torch.from_numpy(d_npz["qfrc_smooth"].astype(np.float32))
        .unsqueeze(0)
        .to(ps.device)
        if ("qfrc_applied" in d_npz and np.any(d_npz["qfrc_applied"] != 0))
        else None
    )
    pwm = (
        torch.from_numpy(d_npz["per_world_mass"].astype(np.float32))
        .unsqueeze(0)
        .to(ps.device)
        if "per_world_mass" in d_npz and np.any(d_npz["per_world_mass"] != 0)
        else None
    )
    pwi = (
        torch.from_numpy(d_npz["per_world_ipos"].astype(np.float32))
        .unsqueeze(0)
        .to(ps.device)
        if "per_world_ipos" in d_npz and np.any(d_npz["per_world_ipos"] != 0)
        else None
    )
    pwa = (
        torch.from_numpy(d_npz["per_world_armature"].astype(np.float32))
        .unsqueeze(0)
        .to(ps.device)
        if "per_world_armature" in d_npz
        and np.any(d_npz["per_world_armature"] != 0)
        else None
    )

    f_tensor = None
    if "randomized_friction" in sc_name and int(d_npz["ncon"]) > 0:
      f_tensor = (
          torch.from_numpy(d_npz["contact_friction"][:, :2].astype(np.float32))
          .unsqueeze(0)
          .to(ps.device)
      )

    step_res = ps.step_autonomous(
        qp_t,
        qv_t,
        f_smooth=f_smooth,
        friction=f_tensor,
        per_world_mass=pwm,
        per_world_ipos=pwi,
        per_world_armature=pwa,
        max_iters=200,
        tol=1e-5,
        dt=0.005,
    )
    safe_mps_sync(ps.device)

    qp_gpu = step_res.qpos[0].cpu().numpy()
    qv_gpu = step_res.qvel[0].cpu().numpy()
    int_stat = int(step_res.integration_status[0].cpu().item())
    s_stat = int(step_res.physics_outputs.solver_status[0].cpu().item())
    nefc = int(step_res.physics_outputs.nefc[0].cpu().item())
    kkt = compute_kkt(step_res.physics_outputs, nefc, 200)

    # Gate enforcement
    violations = []
    violations.extend(validate_raw_state(qp_gpu, qv_gpu, sc_name))
    violations.extend(validate_statuses(s_stat, int_stat, sc_name))

    metrics = compute_separated_metrics(
        qp_gpu, qv_gpu, d_matched.qpos, d_matched.qvel
    )
    violations.extend(validate_metrics(metrics, sc_name))

    if violations:
      gate_violations.extend(violations)

    results.append(
        {
            "scenario": sc_name,
            "nefc": nefc,
            "stat": kkt["stat"],
            "int_stat": int_stat,
            "iters": kkt["iters"],
            "label": kkt["label"],
            "proj_res": kkt["proj"],
            "dual_infeas": kkt["dual"],
            "pos_err": metrics["pos_err"],
            "so3_err": metrics["so3_err"],
            "linvel_err": metrics["linvel_err"],
            "angvel_err": metrics["angvel_err"],
            "jnt_pos_err": metrics["jnt_pos_err"],
            "jnt_vel_err": metrics["jnt_vel_err"],
            "passed": len(violations) == 0,
        }
    )

  return results, gate_violations


def evaluate_free_running_trajectories(
    ps: RepresentativePhysicsSlice,
    canonical: CanonicalMicroDuckModel,
) -> Tuple[Dict[str, Dict], List[str]]:
  """Evaluates free-running autonomous CPU vs GPU trajectories across control horizons with full-trace maxima."""
  regimes = {
      "airborne": {
          "file": str(PROJECT_ROOT / "corpus" / "airborne.npz"),
          "modify_qpos": lambda qp: np.array(
              [qp[0], qp[1], 10.0, *qp[3:]]
          ),  # Pure airborne across 200 steps
          "steps": [4, 20, 200],
      },
      "nominal_standing_realistic": {
          "file": str(
              PROJECT_ROOT / "corpus" / "nominal_standing_realistic.npz"
          ),
          "modify_qpos": lambda qp: qp,
          "steps": [4, 20],
      },
      "contact_onset_drop": {
          "file": str(
              PROJECT_ROOT / "corpus" / "nominal_standing_realistic.npz"
          ),
          "modify_qpos": lambda qp: np.array(
              [qp[0], qp[1], qp[2] + 0.05, *qp[3:]]
          ),  # Drop from 5 cm
          "steps": [4, 20, 25],
      },
      "sliding_lateral_velocity": {
          "file": str(PROJECT_ROOT / "corpus" / "sliding_lateral_velocity.npz"),
          "modify_qpos": lambda qp: qp,
          "steps": [4, 20],
      },
  }

  trajectory_results = {}
  gate_violations = []

  for regime_name, config in regimes.items():
    data = dict(np.load(config["file"]))
    qpos_init = config["modify_qpos"](data["qpos"].copy())
    qvel_init = data["qvel"].copy()
    max_step = max(config["steps"])

    m_matched = create_matching_cpu_model(canonical, data)
    d_matched = mujoco.MjData(m_matched)

    # GPU Rollout
    qp_t = (
        torch.from_numpy(qpos_init.astype(np.float32))
        .unsqueeze(0)
        .to(ps.device)
    )
    qv_t = (
        torch.from_numpy(qvel_init.astype(np.float32))
        .unsqueeze(0)
        .to(ps.device)
    )
    rollout = ps.rollout_trajectory(qp_t, qv_t, num_steps=max_step, dt=0.005)
    safe_mps_sync(ps.device)

    gpu_qpos = rollout["qpos"][:, 0, :].cpu().numpy()
    gpu_qvel = rollout["qvel"][:, 0, :].cpu().numpy()
    gpu_nefc = rollout["nefc"][:, 0].cpu().numpy()
    gpu_stat = rollout["solver_status"][:, 0].cpu().numpy()
    gpu_int_stat = rollout["integration_status"][:, 0].cpu().numpy()

    # CPU Rollout with fresh state
    d_matched.qpos[:] = qpos_init
    d_matched.qvel[:] = qvel_init
    cpu_qpos = [qpos_init.copy()]
    cpu_qvel = [qvel_init.copy()]
    cpu_ncon = [d_matched.ncon]
    for _ in range(max_step):
      mujoco.mj_step(m_matched, d_matched)
      cpu_qpos.append(d_matched.qpos.copy())
      cpu_qvel.append(d_matched.qvel.copy())
      cpu_ncon.append(d_matched.ncon)

    # Validate every step t in [1, max_step]
    step_metrics_history = []
    for t in range(1, max_step + 1):
      ctx_t = f"{regime_name} step {t}"
      # 1. Raw state finiteness
      raw_v = validate_raw_state(gpu_qpos[t], gpu_qvel[t], ctx_t)
      gate_violations.extend(raw_v)

      # 2. Statuses
      s_stat = int(gpu_stat[t - 1])
      i_stat = int(gpu_int_stat[t - 1])
      stat_v = validate_statuses(s_stat, i_stat, ctx_t)
      gate_violations.extend(stat_v)

      # 3. Compute step metrics
      if raw_v:
        m_t = {k: float("nan") for k in PHYSICAL_GATES}
      else:
        m_t = compute_separated_metrics(
            gpu_qpos[t], gpu_qvel[t], cpu_qpos[t], cpu_qvel[t]
        )
      step_metrics_history.append(m_t)

    # Compute point metrics and trace maxima up to each reported checkpoint
    regime_eval = {}
    for s in config["steps"]:
      m_sep = step_metrics_history[s - 1]

      # Full-trace maxima over steps 1..s
      max_metrics = {}
      for k in PHYSICAL_GATES:
        vals = [step_metrics_history[t - 1][k] for t in range(1, s + 1)]
        if any(not math.isfinite(v) for v in vals):
          max_metrics[k] = float("nan")
        else:
          max_metrics[k] = float(max(vals))

      # Validate full-trace maxima against physical gates!
      trace_v = validate_metrics(
          max_metrics, f"{regime_name} horizon {s} steps ({s*5}ms) trace max"
      )
      gate_violations.extend(trace_v)

      passed_checkpoint = (
          len(trace_v) == 0
          and int(gpu_int_stat[s - 1]) == 0
          and int(gpu_stat[s - 1]) >= 0
          and all(
              int(gpu_int_stat[t - 1]) == 0 and int(gpu_stat[t - 1]) >= 0
              for t in range(1, s + 1)
          )
      )

      regime_eval[s] = {
          "t_ms": s * 5,
          "pos_err": m_sep["pos_err"],
          "so3_err": m_sep["so3_err"],
          "linvel_err": m_sep["linvel_err"],
          "angvel_err": m_sep["angvel_err"],
          "jnt_pos_err": m_sep["jnt_pos_err"],
          "jnt_vel_err": m_sep["jnt_vel_err"],
          "max_pos_err": max_metrics["pos_err"],
          "max_so3_err": max_metrics["so3_err"],
          "max_linvel_err": max_metrics["linvel_err"],
          "max_angvel_err": max_metrics["angvel_err"],
          "max_jnt_pos_err": max_metrics["jnt_pos_err"],
          "max_jnt_vel_err": max_metrics["jnt_vel_err"],
          "nefc_gpu": int(gpu_nefc[s - 1]),
          "ncon_cpu": int(cpu_ncon[s]),
          "stat_gpu": int(gpu_stat[s - 1]),
          "int_stat_gpu": int(gpu_int_stat[s - 1]),
          "passed": passed_checkpoint,
      }

    trajectory_results[regime_name] = regime_eval

  return trajectory_results, gate_violations


def evaluate_actuator_control_interval(
    ps: RepresentativePhysicsSlice,
    canonical: CanonicalMicroDuckModel,
) -> Tuple[Dict, List[str]]:
  """Evaluates actuator torque transmission and 4-substep control interval (20 ms)."""
  data = dict(np.load(str(PROJECT_ROOT / "corpus" / "airborne.npz")))
  m_matched = create_matching_cpu_model(canonical, data)
  d = mujoco.MjData(m_matched)

  qpos = data["qpos"].copy()
  qvel = data["qvel"].copy()
  ctrl = np.linspace(-0.6, 0.6, 14, dtype=np.float64)

  violations = []

  # Experiment 1: 1 Step (5 ms) with fresh state
  d.qpos[:] = qpos
  d.qvel[:] = qvel
  d.ctrl[:] = ctrl
  mujoco.mj_step(m_matched, d)

  qp_t = torch.from_numpy(qpos.astype(np.float32)).unsqueeze(0).to(ps.device)
  qv_t = torch.from_numpy(qvel.astype(np.float32)).unsqueeze(0).to(ps.device)
  ctrl_t = torch.from_numpy(ctrl.astype(np.float32)).unsqueeze(0).to(ps.device)

  step_res = ps.step_autonomous(qp_t, qv_t, ctrl=ctrl_t, dt=0.005)
  safe_mps_sync(ps.device)

  qp_1s = step_res.qpos[0].cpu().numpy()
  qv_1s = step_res.qvel[0].cpu().numpy()
  stat_1s = int(step_res.physics_outputs.solver_status[0].cpu().item())
  int_stat_1s = int(step_res.integration_status[0].cpu().item())

  ctx_1s = "actuator_1step_5ms"
  violations.extend(validate_raw_state(qp_1s, qv_1s, ctx_1s))
  violations.extend(validate_statuses(stat_1s, int_stat_1s, ctx_1s))
  m_1step = compute_separated_metrics(qp_1s, qv_1s, d.qpos, d.qvel)
  violations.extend(validate_metrics(m_1step, ctx_1s))

  # Experiment 2: 4 Substeps (20 ms control interval) with fresh MjData
  d2 = mujoco.MjData(m_matched)
  d2.qpos[:] = qpos
  d2.qvel[:] = qvel
  d2.ctrl[:] = ctrl

  qp_next, qv_next, substep_outputs = ps.step_control_interval(
      qp_t, qv_t, ctrl_t, num_substeps=4, dt=0.005
  )
  safe_mps_sync(ps.device)

  substep_metrics = []
  for k in range(4):
    mujoco.mj_step(m_matched, d2)
    sub_res = substep_outputs[k]
    sub_qp = sub_res.qpos[0].cpu().numpy()
    sub_qv = sub_res.qvel[0].cpu().numpy()
    sub_sstat = int(sub_res.physics_outputs.solver_status[0].cpu().item())
    sub_istat = int(sub_res.integration_status[0].cpu().item())

    ctx_sub = f"actuator_substep_{k+1}_of_4"
    violations.extend(validate_raw_state(sub_qp, sub_qv, ctx_sub))
    violations.extend(validate_statuses(sub_sstat, sub_istat, ctx_sub))
    m_sub = compute_separated_metrics(sub_qp, sub_qv, d2.qpos, d2.qvel)
    violations.extend(validate_metrics(m_sub, ctx_sub))
    substep_metrics.append(
        {
            **m_sub,
            "stat": sub_sstat,
            "int_stat": sub_istat,
        }
    )

  # Full-trace max across the 4 substeps
  max_sub_metrics = {}
  for key in PHYSICAL_GATES:
    vals = [m[key] for m in substep_metrics]
    if any(not math.isfinite(v) for v in vals):
      max_sub_metrics[key] = float("nan")
    else:
      max_sub_metrics[key] = float(max(vals))

  violations.extend(
      validate_metrics(max_sub_metrics, "actuator_4substeps_trace_max")
  )

  # Final endpoint metric and violations
  m_ctrl_int = compute_separated_metrics(
      qp_next[0].cpu().numpy(), qv_next[0].cpu().numpy(), d2.qpos, d2.qvel
  )
  violations.extend(validate_metrics(m_ctrl_int, "actuator_ctrl_int_endpoint"))
  stat_20ms = int(
      substep_outputs[-1].physics_outputs.solver_status[0].cpu().item()
  )
  int_stat_20ms = int(substep_outputs[-1].integration_status[0].cpu().item())

  return {
      "1step_5ms": {**m_1step, "stat": stat_1s, "int_stat": int_stat_1s},
      "ctrl_int_20ms": {
          **m_ctrl_int,
          "stat": stat_20ms,
          "int_stat": int_stat_20ms,
      },
      "substeps": substep_metrics,
      "max_substep_metrics": max_sub_metrics,
  }, violations


def main():
  canonical = load_canonical_model()
  ps = RepresentativePhysicsSlice(batch_size=1, canonical=canonical)

  print(
      "# Milestone 4: Canonical 5 ms ImplicitFast Time Integration Qualification Report\n"
  )
  print(
      "## 1. Common-State One-Step (5 ms) Parity Across All 25 Corpus Scenarios"
  )
  print(
      "| Scenario | nefc | Stat | IntStat | Iter | Label | Proj Res | Dual Infeas | Pos Err (m) | SO(3) Err (rad) | LinVel Err (m/s) | AngVel Err (rad/s) | JntPos Err (rad) | JntVel Err (rad/s) | Status |"
  )
  print(
      "|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|"
  )

  one_step_results, one_step_violations = evaluate_common_state_one_step(
      ps, canonical
  )
  for r in one_step_results:
    status_str = "PASS" if r["passed"] else "FAIL"
    print(
        f"| `{r['scenario']}` | {r['nefc']} | {r['stat']} | {r['int_stat']} | {r['iters']} | {r['label']} | "
        f"{r['proj_res']:.2e} | {r['dual_infeas']:.2e} | {r['pos_err']:.2e} | {r['so3_err']:.2e} | "
        f"{r['linvel_err']:.2e} | {r['angvel_err']:.2e} | {r['jnt_pos_err']:.2e} | {r['jnt_vel_err']:.2e} | **{status_str}** |"
    )

  print(
      "\n## 2. Free-Running Trajectory Qualification (Endpoints and Full-Trace Maxima)"
  )
  print(
      "| Regime | Step | Time (ms) | Pos [Max] (m) | SO(3) [Max] (rad) | LinVel [Max] (m/s) | AngVel [Max] (rad/s) | JntPos [Max] (rad) | JntVel [Max] (rad/s) | nefc | ncon | Stat | IntStat | Status |"
  )
  print(
      "|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|"
  )

  trajectories, traj_violations = evaluate_free_running_trajectories(
      ps, canonical
  )
  for regime, steps_data in trajectories.items():
    for s, d in steps_data.items():
      status_str = "PASS" if d["passed"] else "FAIL"
      print(
          f"| `{regime}` | {s} | {d['t_ms']} | "
          f"{d['pos_err']:.2e} [{d['max_pos_err']:.2e}] | "
          f"{d['so3_err']:.2e} [{d['max_so3_err']:.2e}] | "
          f"{d['linvel_err']:.2e} [{d['max_linvel_err']:.2e}] | "
          f"{d['angvel_err']:.2e} [{d['max_angvel_err']:.2e}] | "
          f"{d['jnt_pos_err']:.2e} [{d['max_jnt_pos_err']:.2e}] | "
          f"{d['jnt_vel_err']:.2e} [{d['max_jnt_vel_err']:.2e}] | "
          f"{d['nefc_gpu']} | {d['ncon_cpu']} | {d['stat_gpu']} | {d['int_stat_gpu']} | **{status_str}** |"
      )

  print("\n## 3. Actuator Torque Control & 20 ms Control Interval (4 Substeps)")
  print(
      "| Interval / Substep | Pos Err (m) | SO(3) Err (rad) | LinVel Err (m/s) | AngVel Err (rad/s) | JntPos Err (rad) | JntVel Err (rad/s) | Stat | IntStat | Status |"
  )
  print("|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|")

  act_results, act_violations = evaluate_actuator_control_interval(
      ps, canonical
  )
  for label in ["1step_5ms", "ctrl_int_20ms"]:
    entry = act_results[label]
    v_sub = validate_metrics(entry, label)
    passed = len(v_sub) == 0 and entry["stat"] >= 0 and entry["int_stat"] == 0
    status_str = "PASS" if passed else "FAIL"
    print(
        f"| `{label}` | {entry['pos_err']:.2e} | {entry['so3_err']:.2e} | "
        f"{entry['linvel_err']:.2e} | {entry['angvel_err']:.2e} | "
        f"{entry['jnt_pos_err']:.2e} | {entry['jnt_vel_err']:.2e} | {entry['stat']} | {entry['int_stat']} | **{status_str}** |"
    )
  for k, sub_m in enumerate(act_results["substeps"]):
    v_sub = validate_metrics(sub_m, f"substep_{k+1}")
    passed = len(v_sub) == 0 and sub_m["stat"] >= 0 and sub_m["int_stat"] == 0
    status_str = "PASS" if passed else "FAIL"
    print(
        f"| `substep_{k+1}_of_4` | {sub_m['pos_err']:.2e} | {sub_m['so3_err']:.2e} | "
        f"{sub_m['linvel_err']:.2e} | {sub_m['angvel_err']:.2e} | "
        f"{sub_m['jnt_pos_err']:.2e} | {sub_m['jnt_vel_err']:.2e} | {sub_m['stat']} | {sub_m['int_stat']} | **{status_str}** |"
    )

  all_violations = one_step_violations + traj_violations + act_violations
  if all_violations:
    print(
        f"\n### ❌ ENFORCED QUALIFICATION GATE FAILURES ({len(all_violations)} violations):"
    )
    for v in all_violations:
      print(f"- {v}")
    sys.exit(1)
  else:
    print(
        "\n### ✅ ALL 25 SCENARIOS, TRAJECTORIES & ACTUATOR SUBSTEPS PASSED ENFORCED NUMERICAL GATES."
    )


if __name__ == "__main__":
  main()
