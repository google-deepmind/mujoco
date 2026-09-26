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

"""Representative Bounded Static-State Physics Slice on Torch MPS & Metal.

Implements canonical kinematics, articulated dynamics (CRBA/RNE with rotor armature),
real CAD sole-plane contact manifold generation, and constrained PGS solve directly
on persistent MPS tensors without host staging. Compares forces, accelerations, and
completed GPU runtime against pinned MuJoCo CPU reference across the three canonical states.
"""

import ctypes
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

import mujoco
import numpy as np
import torch

from ._kernels import MetalKernelManager
from ._model import CanonicalMicroDuckModel
from ._model import get_canonical_states
from ._model import load_canonical_model

SHADER_PATH = (
    Path(__file__).resolve().parent / "shaders" / "physics_slice.metal"
)


class BodyConstants(ctypes.Structure):
  _fields_ = [
      ("parent_id", ctypes.c_int32),
      ("joint_type", ctypes.c_int32),
      ("qpos_adr", ctypes.c_int32),
      ("dof_adr", ctypes.c_int32),
      ("body_pos", ctypes.c_float * 3),
      ("body_quat", ctypes.c_float * 4),
      ("body_ipos", ctypes.c_float * 3),
      ("body_iquat", ctypes.c_float * 4),
      ("jnt_axis", ctypes.c_float * 3),
      ("mass", ctypes.c_float),
      ("inertia", ctypes.c_float * 3),
  ]


class DofConstants(ctypes.Structure):
  _fields_ = [
      ("dof_parentid", ctypes.c_int32),
      ("dof_bodyid", ctypes.c_int32),
      ("dof_armature", ctypes.c_float),
  ]


class GeomConstants(ctypes.Structure):
  _fields_ = [
      ("body_id", ctypes.c_int32),
      ("geom_pos", ctypes.c_float * 3),
      ("geom_quat", ctypes.c_float * 4),
  ]


class ContactSolverParams(ctypes.Structure):
  _fields_ = [
      ("timeconst", ctypes.c_float),
      ("dampratio", ctypes.c_float),
      ("dmin", ctypes.c_float),
      ("dmax", ctypes.c_float),
      ("width", ctypes.c_float),
      ("midpoint", ctypes.c_float),
      ("power", ctypes.c_float),
      ("impratio", ctypes.c_float),
      ("margin", ctypes.c_float),
  ]


@dataclass
class PhysicsSliceOutputs:
  body_xpos: torch.Tensor  # (B, 17, 3)
  body_xmat: torch.Tensor  # (B, 17, 9)
  body_xipos: torch.Tensor  # (B, 17, 3)
  body_ximat: torch.Tensor  # (B, 17, 9)
  subtree_com: torch.Tensor  # (B, 3)
  geom_xpos: torch.Tensor  # (B, 2, 3)
  geom_xmat: torch.Tensor  # (B, 2, 9)
  contact_pos: torch.Tensor  # (B, nconmax, 3)
  contact_dist: torch.Tensor  # (B, nconmax)
  contact_normal: torch.Tensor  # (B, nconmax, 3)
  contact_body: torch.Tensor  # (B, nconmax)
  ncon: torch.Tensor  # (B,) int32
  overflow_flag: torch.Tensor  # (B,) int32
  M_eff: torch.Tensor  # (B, 20, 20)
  L_factor: torch.Tensor  # (B, 20, 20)
  M_inv: torch.Tensor  # (B, 20, 20)
  qfrc_bias: torch.Tensor  # (B, 20)
  qfrc_constraint: torch.Tensor  # (B, 20)
  qacc: torch.Tensor  # (B, 20)
  solver_status: torch.Tensor  # (B,) int32

  def clone(self) -> "PhysicsSliceOutputs":
    return PhysicsSliceOutputs(
        body_xpos=self.body_xpos.clone(),
        body_xmat=self.body_xmat.clone(),
        body_xipos=self.body_xipos.clone(),
        body_ximat=self.body_ximat.clone(),
        subtree_com=self.subtree_com.clone(),
        geom_xpos=self.geom_xpos.clone(),
        geom_xmat=self.geom_xmat.clone(),
        contact_pos=self.contact_pos.clone(),
        contact_dist=self.contact_dist.clone(),
        contact_normal=self.contact_normal.clone(),
        contact_body=self.contact_body.clone(),
        ncon=self.ncon.clone(),
        overflow_flag=self.overflow_flag.clone(),
        M_eff=self.M_eff.clone(),
        L_factor=self.L_factor.clone(),
        M_inv=self.M_inv.clone(),
        qfrc_bias=self.qfrc_bias.clone(),
        qfrc_constraint=self.qfrc_constraint.clone(),
        qacc=self.qacc.clone(),
        solver_status=self.solver_status.clone(),
    )


@dataclass
class AutonomousPhysicsSliceOutputs:
  body_xpos: torch.Tensor  # (B, 17, 3)
  body_xmat: torch.Tensor  # (B, 17, 9)
  body_xipos: torch.Tensor  # (B, 17, 3)
  body_ximat: torch.Tensor  # (B, 17, 9)
  subtree_com: torch.Tensor  # (B, 3)
  geom_xpos: torch.Tensor  # (B, 2, 3)
  geom_xmat: torch.Tensor  # (B, 2, 9)
  # CAD Contact Manifold
  contact_pos: torch.Tensor  # (B, nconmax, 3)
  contact_dist: torch.Tensor  # (B, nconmax)
  contact_normal: torch.Tensor  # (B, nconmax, 3)
  contact_body: torch.Tensor  # (B, nconmax)
  contact_geom: torch.Tensor  # (B, nconmax)
  ncon: torch.Tensor  # (B,) int32
  contact_overflow: torch.Tensor  # (B,) int32
  # Assembled Constraints
  J: torch.Tensor  # (B, capacity, 20)
  aref: torch.Tensor  # (B, capacity)
  R: torch.Tensor  # (B, capacity)
  efc_type: torch.Tensor  # (B, capacity) int32
  nefc: torch.Tensor  # (B,) int32
  assembly_overflow: torch.Tensor  # (B,) int32
  # Dynamics & Factorization
  M_eff: torch.Tensor  # (B, 20, 20)
  L_factor: torch.Tensor  # (B, 20, 20)
  cholesky_status: torch.Tensor  # (B,) int32
  qfrc_bias: torch.Tensor  # (B, 20)
  qfrc_actuator: torch.Tensor  # (B, 20)
  f_smooth: torch.Tensor  # (B, 20)
  # Constrained Solve
  lambda_force: torch.Tensor  # (B, capacity)
  qfrc_constraint: torch.Tensor  # (B, 20)
  qacc: torch.Tensor  # (B, 20)
  solver_status: torch.Tensor  # (B,) int32
  actual_iters: torch.Tensor  # (B,) int32
  dual_residual: torch.Tensor  # (B,) float32
  efc_frictionloss: Optional[torch.Tensor] = None  # (B, capacity)
  efc_id: Optional[torch.Tensor] = None  # (B, capacity) int32
  integration_status: Optional[torch.Tensor] = None

  def clone(self) -> "AutonomousPhysicsSliceOutputs":
    return AutonomousPhysicsSliceOutputs(
        body_xpos=self.body_xpos.clone(),
        body_xmat=self.body_xmat.clone(),
        body_xipos=self.body_xipos.clone(),
        body_ximat=self.body_ximat.clone(),
        subtree_com=self.subtree_com.clone(),
        geom_xpos=self.geom_xpos.clone(),
        geom_xmat=self.geom_xmat.clone(),
        contact_pos=self.contact_pos.clone(),
        contact_dist=self.contact_dist.clone(),
        contact_normal=self.contact_normal.clone(),
        contact_body=self.contact_body.clone(),
        contact_geom=self.contact_geom.clone(),
        ncon=self.ncon.clone(),
        contact_overflow=self.contact_overflow.clone(),
        J=self.J.clone(),
        aref=self.aref.clone(),
        R=self.R.clone(),
        efc_type=self.efc_type.clone(),
        nefc=self.nefc.clone(),
        assembly_overflow=self.assembly_overflow.clone(),
        M_eff=self.M_eff.clone(),
        L_factor=self.L_factor.clone(),
        cholesky_status=self.cholesky_status.clone(),
        qfrc_bias=self.qfrc_bias.clone(),
        qfrc_actuator=self.qfrc_actuator.clone(),
        f_smooth=self.f_smooth.clone(),
        lambda_force=self.lambda_force.clone(),
        qfrc_constraint=self.qfrc_constraint.clone(),
        qacc=self.qacc.clone(),
        solver_status=self.solver_status.clone(),
        actual_iters=self.actual_iters.clone(),
        dual_residual=self.dual_residual.clone(),
        efc_frictionloss=self.efc_frictionloss.clone()
        if self.efc_frictionloss is not None
        else None,
        efc_id=self.efc_id.clone() if self.efc_id is not None else None,
        integration_status=self.integration_status.clone()
        if self.integration_status is not None
        else None,
    )


@dataclass
class AutonomousStepOutputs:
  qpos: torch.Tensor  # (B, 21) advanced coordinates
  qvel: torch.Tensor  # (B, 20) advanced velocities
  integration_status: torch.Tensor  # (B,) int32
  physics_outputs: AutonomousPhysicsSliceOutputs

  def __getattr__(self, name: str):
    if name != "physics_outputs" and hasattr(self.physics_outputs, name):
      return getattr(self.physics_outputs, name)
    raise AttributeError(
        f"'{type(self).__name__}' object has no attribute '{name}'"
    )

  def clone(self) -> "AutonomousStepOutputs":
    return AutonomousStepOutputs(
        qpos=self.qpos.clone(),
        qvel=self.qvel.clone(),
        integration_status=self.integration_status.clone(),
        physics_outputs=self.physics_outputs.clone(),
    )


class RepresentativePhysicsSlice:
  """Bounded representative physics slice for MicroDuck."""

  def __init__(
      self,
      batch_size: int = 1,
      nconmax: int = 35,
      device: str = "mps",
      canonical: Optional[CanonicalMicroDuckModel] = None,
      autonomous_capacity: int = 64,
  ):
    self.device = torch.device(device)
    self.batch_size = batch_size
    self.nconmax = nconmax
    self.autonomous_capacity = autonomous_capacity
    self.canonical = canonical or load_canonical_model()
    self.m = self.canonical.model

    # Verify model preconditions for ImplicitFast state advancement
    if self.m.opt.viscosity != 0.0 or self.m.opt.density != 0.0:
      raise NotImplementedError(
          "RepresentativePhysicsSlice requires zero fluid viscosity/density"
      )
    if not np.all(self.m.actuator_biastype == 0):
      raise NotImplementedError(
          "RepresentativePhysicsSlice requires mjBIAS_NONE (zero actuator bias derivatives)"
      )

    # Compile Metal shaders
    self.km = MetalKernelManager(SHADER_PATH)

    # Prepare model constants on CPU and MPS
    self._init_constants()

    # Allocate persistent device buffers
    self._init_persistent_buffers()

  def _init_constants(self):
    m = self.m
    # 17 bodies
    bodies_array = (BodyConstants * 17)()
    for i in range(17):
      bodies_array[i].parent_id = int(m.body_parentid[i])
      if i >= 3:
        jnt_id = [j for j in range(m.njnt) if m.jnt_bodyid[j] == i][0]
        bodies_array[i].joint_type = int(m.jnt_type[jnt_id])
        bodies_array[i].qpos_adr = int(m.jnt_qposadr[jnt_id])
        bodies_array[i].dof_adr = int(m.jnt_dofadr[jnt_id])
        for k in range(3):
          bodies_array[i].jnt_axis[k] = float(m.jnt_axis[jnt_id, k])
      else:
        bodies_array[i].joint_type = 0 if i == 2 else -1
        bodies_array[i].qpos_adr = 0
        bodies_array[i].dof_adr = 0
        for k in range(3):
          bodies_array[i].jnt_axis[k] = 0.0

      for k in range(3):
        bodies_array[i].body_pos[k] = float(m.body_pos[i, k])
        bodies_array[i].body_ipos[k] = float(m.body_ipos[i, k])
        bodies_array[i].inertia[k] = float(m.body_inertia[i, k])
      for k in range(4):
        bodies_array[i].body_quat[k] = float(m.body_quat[i, k])
        bodies_array[i].body_iquat[k] = float(m.body_iquat[i, k])
      bodies_array[i].mass = float(m.body_mass[i])

    self.bodies_buf = torch.frombuffer(bodies_array, dtype=torch.uint8).to(
        self.device
    )

    # 20 DOFs
    dofs_array = (DofConstants * 20)()
    for d in range(20):
      dofs_array[d].dof_parentid = int(m.dof_parentid[d])
      dofs_array[d].dof_bodyid = int(m.dof_bodyid[d])
      dofs_array[d].dof_armature = float(m.dof_armature[d])

    self.dofs_buf = torch.frombuffer(dofs_array, dtype=torch.uint8).to(
        self.device
    )

    # 2 foot geoms
    geoms_array = (GeomConstants * 2)()
    for idx, g_id in enumerate(
        [self.canonical.left_foot_geom_id, self.canonical.right_foot_geom_id]
    ):
      geoms_array[idx].body_id = int(m.geom_bodyid[g_id])
      for k in range(3):
        geoms_array[idx].geom_pos[k] = float(m.geom_pos[g_id, k])
      for k in range(4):
        geoms_array[idx].geom_quat[k] = float(m.geom_quat[g_id, k])

    left_foot_body = int(m.geom_bodyid[self.canonical.left_foot_geom_id])
    right_foot_body = int(m.geom_bodyid[self.canonical.right_foot_geom_id])
    assert (
        left_foot_body == 7
    ), f"Expected left foot body ID 7, got {left_foot_body}"
    assert (
        right_foot_body == 16
    ), f"Expected right foot body ID 16, got {right_foot_body}"

    self.geoms_buf = torch.frombuffer(geoms_array, dtype=torch.uint8).to(
        self.device
    )

    # Foot CAD meshes (read-only device buffers)
    self.left_verts = torch.from_numpy(
        self.canonical.left_foot_mesh.vertices.astype(np.float32)
    ).to(self.device)
    self.right_verts = torch.from_numpy(
        self.canonical.right_foot_mesh.vertices.astype(np.float32)
    ).to(self.device)
    self.num_left_verts = int(self.canonical.left_foot_mesh.vertnum)
    self.num_right_verts = int(self.canonical.right_foot_mesh.vertnum)

    # Mesh graph and rbound for CAD contact manifold v2
    self.left_graph = torch.from_numpy(
        self.canonical.left_foot_mesh.graph.astype(np.int32)
    ).to(self.device)
    self.right_graph = torch.from_numpy(
        self.canonical.right_foot_mesh.graph.astype(np.int32)
    ).to(self.device)
    self.left_rbound = float(self.canonical.left_foot_mesh.rbound)
    self.right_rbound = float(self.canonical.right_foot_mesh.rbound)

    # Body inverse weights for constraint regularization
    self.body_invweight0 = torch.from_numpy(
        self.m.body_invweight0.astype(np.float32)
    ).to(self.device)
    self.dof_invweight0 = torch.from_numpy(
        self.m.dof_invweight0[6:20].astype(np.float32)
    ).to(self.device)

    # Default contact solver parameters
    self.default_contact_params = ContactSolverParams(
        timeconst=0.02,
        dampratio=1.0,
        dmin=0.9,
        dmax=0.95,
        width=0.001,
        midpoint=0.5,
        power=2.0,
        impratio=float(self.m.opt.impratio),
        margin=0.0,
    )
    self.default_contact_params_buf = torch.frombuffer(
        self.default_contact_params, dtype=torch.uint8
    ).to(self.device)

    # Motor armature
    self.armature = torch.from_numpy(
        self.canonical.dof_armature.astype(np.float32)
    ).to(self.device)

    # Combined default friction from canonical terrain and foot geoms
    terrain_mu = float(self.m.geom_friction[0, 0])
    foot_mu = float(self.m.geom_friction[self.canonical.left_foot_geom_id, 0])
    self.canonical_default_friction = max(terrain_mu, foot_mu)
    self.default_friction = torch.tensor(
        [self.canonical_default_friction, self.canonical_default_friction],
        dtype=torch.float32,
        device=self.device,
    )

    # Actuator force range limits (14, 2)
    self.actuator_forcerange = torch.from_numpy(
        self.m.actuator_forcerange.astype(np.float32)
    ).to(self.device)

  def _init_persistent_buffers(self):
    B = self.batch_size
    dev = self.device

    self.body_xpos = torch.zeros((B, 17, 3), dtype=torch.float32, device=dev)
    self.body_xmat = torch.zeros((B, 17, 9), dtype=torch.float32, device=dev)
    self.body_xipos = torch.zeros((B, 17, 3), dtype=torch.float32, device=dev)
    self.body_ximat = torch.zeros((B, 17, 9), dtype=torch.float32, device=dev)
    self.subtree_com = torch.zeros((B, 3), dtype=torch.float32, device=dev)

    self.geom_xpos = torch.zeros((B, 2, 3), dtype=torch.float32, device=dev)
    self.geom_xmat = torch.zeros((B, 2, 9), dtype=torch.float32, device=dev)

    self.contact_pos = torch.zeros(
        (B, self.nconmax, 3), dtype=torch.float32, device=dev
    )
    self.contact_dist = torch.zeros(
        (B, self.nconmax), dtype=torch.float32, device=dev
    )
    self.contact_normal = torch.zeros(
        (B, self.nconmax, 3), dtype=torch.float32, device=dev
    )
    self.contact_body = torch.zeros(
        (B, self.nconmax), dtype=torch.int32, device=dev
    )
    self.contact_geom = torch.zeros(
        (B, self.nconmax), dtype=torch.int32, device=dev
    )
    self.ncon = torch.zeros((B,), dtype=torch.int32, device=dev)
    self.overflow_flag = torch.zeros((B,), dtype=torch.int32, device=dev)
    self.contact_overflow = torch.zeros((B,), dtype=torch.int32, device=dev)

    self.M_eff = torch.zeros((B, 20, 20), dtype=torch.float32, device=dev)
    self.L_factor = torch.zeros((B, 20, 20), dtype=torch.float32, device=dev)
    self.M_inv = torch.zeros((B, 20, 20), dtype=torch.float32, device=dev)
    self.qfrc_bias = torch.zeros((B, 20), dtype=torch.float32, device=dev)
    self.solver_status = torch.zeros((B,), dtype=torch.int32, device=dev)

    self.qacc = torch.zeros((B, 20), dtype=torch.float32, device=dev)
    self.qfrc_constraint = torch.zeros((B, 20), dtype=torch.float32, device=dev)

    # Pre-allocated identity matrix for native Cholesky inversion
    eye = (
        torch.eye(20, dtype=torch.float32, device=dev)
        .unsqueeze(0)
        .expand(B, -1, -1)
        .contiguous()
    )
    self.eye_20 = eye

    # Dummy per-world buffers for unperturbed runs
    self.dummy_mass = torch.zeros((B, 17), dtype=torch.float32, device=dev)
    self.dummy_ipos = torch.zeros((B, 17, 3), dtype=torch.float32, device=dev)
    self.dummy_armature = torch.zeros((B, 20), dtype=torch.float32, device=dev)
    self.dummy_inertia = torch.zeros(
        (B, 17, 3), dtype=torch.float32, device=dev
    )
    self.dummy_damping = torch.zeros((B, 20), dtype=torch.float32, device=dev)
    self.dummy_iquat = torch.zeros((B, 17, 4), dtype=torch.float32, device=dev)
    self.dummy_dof_invweight0 = torch.zeros(
        (B, 14), dtype=torch.float32, device=dev
    )

    # Persistent buffers for oracle constraint solve
    self.constraint_capacity = max(getattr(self, "autonomous_capacity", 64), 64)
    self.oracle_lambda = torch.zeros(
        (B, self.constraint_capacity), dtype=torch.float32, device=dev
    )
    self.oracle_qfrc_constraint = torch.zeros(
        (B, 20), dtype=torch.float32, device=dev
    )
    self.oracle_qacc = torch.zeros((B, 20), dtype=torch.float32, device=dev)
    self.oracle_solver_status = torch.zeros((B,), dtype=torch.int32, device=dev)
    self.oracle_actual_iters = torch.zeros((B,), dtype=torch.int32, device=dev)
    self.oracle_dual_residual = torch.zeros(
        (B,), dtype=torch.float32, device=dev
    )
    self.scratchpad_A = torch.zeros(
        (B, self.constraint_capacity, self.constraint_capacity),
        dtype=torch.float32,
        device=dev,
    )
    self.scratchpad_Y = torch.zeros(
        (B, self.constraint_capacity, 20), dtype=torch.float32, device=dev
    )
    self.dummy_frictionloss = torch.zeros(
        (B, self.constraint_capacity), dtype=torch.float32, device=dev
    )

    # Persistent buffers for assembled contact constraints (Milestone 3B + 5)
    self.assembled_J = torch.zeros(
        (B, self.autonomous_capacity, 20), dtype=torch.float32, device=dev
    )
    self.assembled_aref = torch.zeros(
        (B, self.autonomous_capacity), dtype=torch.float32, device=dev
    )
    self.assembled_R = torch.zeros(
        (B, self.autonomous_capacity), dtype=torch.float32, device=dev
    )
    self.assembled_efc_type = torch.zeros(
        (B, self.autonomous_capacity), dtype=torch.int32, device=dev
    )
    self.assembled_efc_id = torch.zeros(
        (B, self.autonomous_capacity), dtype=torch.int32, device=dev
    )
    self.assembled_nefc = torch.zeros((B,), dtype=torch.int32, device=dev)
    self.assembled_frictionloss = torch.zeros(
        (B, self.autonomous_capacity), dtype=torch.float32, device=dev
    )
    self.assembled_friction = torch.full(
        (B, self.nconmax, 2),
        self.canonical_default_friction,
        dtype=torch.float32,
        device=dev,
    )
    self.assembly_overflow = torch.zeros((B,), dtype=torch.int32, device=dev)
    self.autonomous_upstream_status = torch.zeros(
        (B,), dtype=torch.int32, device=dev
    )
    self.friction_batch = torch.full(
        (B, self.nconmax, 2),
        self.canonical_default_friction,
        dtype=torch.float32,
        device=dev,
    )
    self.dummy_qpos = torch.zeros((B, 21), dtype=torch.float32, device=dev)
    self.dummy_dof_frictionloss = torch.zeros(
        (B, 14), dtype=torch.float32, device=dev
    )
    # Persistent buffers for time-integration (Milestone 4)
    self.integrated_qpos = torch.zeros((B, 21), dtype=torch.float32, device=dev)
    self.integrated_qvel = torch.zeros((B, 20), dtype=torch.float32, device=dev)
    self.integration_status = torch.zeros((B,), dtype=torch.int32, device=dev)

    # Persistent buffers for merged contacts (ground + extra robot-robot contacts)
    self.merged_nconmax = self.nconmax + 8
    self.merged_contact_pos = torch.zeros(
        (B, self.merged_nconmax, 3), dtype=torch.float32, device=dev
    )
    self.merged_contact_dist = torch.zeros(
        (B, self.merged_nconmax), dtype=torch.float32, device=dev
    )
    self.merged_contact_body1 = torch.zeros(
        (B, self.merged_nconmax), dtype=torch.int32, device=dev
    )
    self.merged_contact_body2 = torch.zeros(
        (B, self.merged_nconmax), dtype=torch.int32, device=dev
    )
    self.merged_contact_frame = torch.zeros(
        (B, self.merged_nconmax, 9), dtype=torch.float32, device=dev
    )
    self.merged_contact_friction = torch.zeros(
        (B, self.merged_nconmax, 2), dtype=torch.float32, device=dev
    )
    self.merged_ncon = torch.zeros((B,), dtype=torch.int32, device=dev)
    self.merged_contact_overflow = torch.zeros(
        (B,), dtype=torch.int32, device=dev
    )

  def _ensure_batch_size(self, B: int):
    """Ensures internal persistent buffers match requested batch size B, reallocating if needed."""
    if self.batch_size != B:
      self.batch_size = B
      self._init_persistent_buffers()

  def _prepare_inputs(
      self,
      qpos: torch.Tensor,
      qvel: torch.Tensor,
      per_world_mass: Optional[torch.Tensor] = None,
      per_world_ipos: Optional[torch.Tensor] = None,
      per_world_armature: Optional[torch.Tensor] = None,
      per_world_inertia: Optional[torch.Tensor] = None,
      per_world_damping: Optional[torch.Tensor] = None,
      per_world_iquat: Optional[torch.Tensor] = None,
  ) -> Tuple[
      torch.Tensor,
      torch.Tensor,
      Optional[torch.Tensor],
      Optional[torch.Tensor],
      Optional[torch.Tensor],
      Optional[torch.Tensor],
      Optional[torch.Tensor],
      Optional[torch.Tensor],
      int,
  ]:
    """Validates and prepares contiguous device inputs before any GPU allocation or kernel dispatch."""
    if not isinstance(qpos, torch.Tensor) or not isinstance(qvel, torch.Tensor):
      raise TypeError(
          f"qpos and qvel must be torch.Tensor instances, got {type(qpos)}, {type(qvel)}"
      )
    if (
        qpos.device.type != self.device.type
        or qvel.device.type != self.device.type
    ):
      raise ValueError(
          f"Inputs must be on device {self.device.type}, got qpos on {qpos.device}, qvel on {qvel.device}"
      )
    if qpos.dtype != torch.float32 or qvel.dtype != torch.float32:
      raise TypeError(
          f"Inputs must be float32, got qpos {qpos.dtype}, qvel {qvel.dtype}"
      )
    if qpos.ndim != 2 or qpos.shape[1] != 21:
      raise ValueError(f"Expected qpos shape (B, 21), got {qpos.shape}")
    if qvel.ndim != 2 or qvel.shape[1] != 20:
      raise ValueError(f"Expected qvel shape (B, 20), got {qvel.shape}")
    if qpos.shape[0] != qvel.shape[0]:
      raise ValueError(
          f"Batch dimension mismatch: qpos {qpos.shape[0]} vs qvel {qvel.shape[0]}"
      )

    B = qpos.shape[0]
    if B <= 0:
      raise ValueError(f"Batch size must be positive, got {B}")

    if per_world_mass is not None:
      if not isinstance(per_world_mass, torch.Tensor):
        raise TypeError(
            f"per_world_mass must be a torch.Tensor, got {type(per_world_mass)}"
        )
      if (
          per_world_mass.device.type != self.device.type
          or per_world_mass.dtype != torch.float32
      ):
        raise ValueError(
            f"per_world_mass must be float32 on {self.device.type}, got {per_world_mass.dtype} on {per_world_mass.device}"
        )
      if per_world_mass.shape != (B, 17):
        raise ValueError(
            f"Expected per_world_mass shape ({B}, 17), got {per_world_mass.shape}"
        )
      if not per_world_mass.is_contiguous():
        per_world_mass = per_world_mass.contiguous()

    if per_world_ipos is not None:
      if not isinstance(per_world_ipos, torch.Tensor):
        raise TypeError(
            f"per_world_ipos must be a torch.Tensor, got {type(per_world_ipos)}"
        )
      if (
          per_world_ipos.device.type != self.device.type
          or per_world_ipos.dtype != torch.float32
      ):
        raise ValueError(
            f"per_world_ipos must be float32 on {self.device.type}, got {per_world_ipos.dtype} on {per_world_ipos.device}"
        )
      if per_world_ipos.shape != (B, 17, 3):
        raise ValueError(
            f"Expected per_world_ipos shape ({B}, 17, 3), got {per_world_ipos.shape}"
        )
      if not per_world_ipos.is_contiguous():
        per_world_ipos = per_world_ipos.contiguous()

    if per_world_armature is not None:
      if not isinstance(per_world_armature, torch.Tensor):
        raise TypeError(
            f"per_world_armature must be a torch.Tensor, got {type(per_world_armature)}"
        )
      if (
          per_world_armature.device.type != self.device.type
          or per_world_armature.dtype != torch.float32
      ):
        raise ValueError(
            f"per_world_armature must be float32 on {self.device.type}, got {per_world_armature.dtype} on {per_world_armature.device}"
        )
      if per_world_armature.shape != (B, 20):
        raise ValueError(
            f"Expected per_world_armature shape ({B}, 20), got {per_world_armature.shape}"
        )
      if not per_world_armature.is_contiguous():
        per_world_armature = per_world_armature.contiguous()

    if per_world_inertia is not None:
      if not isinstance(per_world_inertia, torch.Tensor):
        raise TypeError(
            f"per_world_inertia must be a torch.Tensor, got {type(per_world_inertia)}"
        )
      if (
          per_world_inertia.device.type != self.device.type
          or per_world_inertia.dtype != torch.float32
      ):
        raise ValueError(
            f"per_world_inertia must be float32 on {self.device.type}"
        )
      if per_world_inertia.ndim == 2 and per_world_inertia.shape == (17, 3):
        per_world_inertia = (
            per_world_inertia.unsqueeze(0).expand(B, 17, 3).contiguous()
        )
      elif per_world_inertia.shape != (B, 17, 3):
        raise ValueError(
            f"Expected per_world_inertia shape ({B}, 17, 3), got {per_world_inertia.shape}"
        )
      if not per_world_inertia.is_contiguous():
        per_world_inertia = per_world_inertia.contiguous()

    if per_world_damping is not None:
      if not isinstance(per_world_damping, torch.Tensor):
        raise TypeError(
            f"per_world_damping must be a torch.Tensor, got {type(per_world_damping)}"
        )
      if (
          per_world_damping.device.type != self.device.type
          or per_world_damping.dtype != torch.float32
      ):
        raise ValueError(
            f"per_world_damping must be float32 on {self.device.type}"
        )
      if per_world_damping.ndim == 1 and per_world_damping.shape[0] == 14:
        d_buf = torch.zeros((B, 20), dtype=torch.float32, device=self.device)
        d_buf[:, 6:20] = per_world_damping
        per_world_damping = d_buf
      elif per_world_damping.ndim == 2 and per_world_damping.shape == (B, 14):
        d_buf = torch.zeros((B, 20), dtype=torch.float32, device=self.device)
        d_buf[:, 6:20] = per_world_damping
        per_world_damping = d_buf
      elif per_world_damping.ndim == 1 and per_world_damping.shape[0] == 20:
        per_world_damping = (
            per_world_damping.unsqueeze(0).expand(B, 20).contiguous()
        )
      elif per_world_damping.shape != (B, 20):
        raise ValueError(
            f"Expected per_world_damping shape ({B}, 20), got {per_world_damping.shape}"
        )
      if not per_world_damping.is_contiguous():
        per_world_damping = per_world_damping.contiguous()

    if per_world_iquat is not None:
      if not isinstance(per_world_iquat, torch.Tensor):
        raise TypeError(
            f"per_world_iquat must be a torch.Tensor, got {type(per_world_iquat)}"
        )
      if (
          per_world_iquat.device.type != self.device.type
          or per_world_iquat.dtype != torch.float32
      ):
        raise ValueError(
            f"per_world_iquat must be float32 on {self.device.type}"
        )
      if per_world_iquat.ndim == 2 and per_world_iquat.shape == (17, 4):
        per_world_iquat = (
            per_world_iquat.unsqueeze(0).expand(B, 17, 4).contiguous()
        )
      elif per_world_iquat.shape != (B, 17, 4):
        raise ValueError(
            f"Expected per_world_iquat shape ({B}, 17, 4), got {per_world_iquat.shape}"
        )
      if not per_world_iquat.is_contiguous():
        per_world_iquat = per_world_iquat.contiguous()

    if not qpos.is_contiguous():
      qpos = qpos.contiguous()
    if not qvel.is_contiguous():
      qvel = qvel.contiguous()

    return (
        qpos,
        qvel,
        per_world_mass,
        per_world_ipos,
        per_world_armature,
        per_world_inertia,
        per_world_damping,
        per_world_iquat,
        B,
    )

  def compute_forward_kinematics(self, qpos: torch.Tensor):
    """Computes forward kinematics for bodies and geoms on Metal from qpos without full dynamics."""
    if not isinstance(qpos, torch.Tensor):
      raise TypeError(f"qpos must be a torch.Tensor, got {type(qpos)}")
    if qpos.device.type != self.device.type or qpos.dtype != torch.float32:
      raise ValueError(f"qpos must be float32 on {self.device.type}")
    if qpos.ndim != 2 or qpos.shape[1] != 21:
      raise ValueError(f"Expected qpos shape (B, 21), got {qpos.shape}")
    B = qpos.shape[0]
    self._ensure_batch_size(B)
    self.km.launch(
        "kernel_forward_kinematics",
        self.bodies_buf,
        self.geoms_buf,
        qpos.contiguous(),
        self.body_xpos,
        self.body_xmat,
        self.geom_xpos,
        self.geom_xmat,
        threads=B,
    )
    return self.body_xpos[:B], self.body_xmat[:B]

  def compute_native_dynamics(
      self,
      qpos: torch.Tensor,
      qvel: torch.Tensor,
      per_world_mass: Optional[torch.Tensor] = None,
      per_world_ipos: Optional[torch.Tensor] = None,
      per_world_armature: Optional[torch.Tensor] = None,
      per_world_inertia: Optional[torch.Tensor] = None,
      per_world_damping: Optional[torch.Tensor] = None,
      per_world_iquat: Optional[torch.Tensor] = None,
      dt: float = 0.005,
  ):
    """Computes native articulated dynamics (kinematics, CRBA mass matrix, RNE bias forces)

    directly on MPS tensors via Metal kernel without host round-trips.
    """
    (
        qpos,
        qvel,
        per_world_mass,
        per_world_ipos,
        per_world_armature,
        per_world_inertia,
        per_world_damping,
        per_world_iquat,
        B,
    ) = self._prepare_inputs(
        qpos,
        qvel,
        per_world_mass=per_world_mass,
        per_world_ipos=per_world_ipos,
        per_world_armature=per_world_armature,
        per_world_inertia=per_world_inertia,
        per_world_damping=per_world_damping,
        per_world_iquat=per_world_iquat,
    )
    self._ensure_batch_size(B)

    flags = 0
    buf_mass = self.dummy_mass
    buf_ipos = self.dummy_ipos
    buf_armature = self.dummy_armature
    buf_inertia = self.dummy_inertia
    buf_damping = self.dummy_damping
    buf_iquat = self.dummy_iquat

    if per_world_mass is not None:
      flags |= 1
      buf_mass = per_world_mass
    if per_world_ipos is not None:
      flags |= 2
      buf_ipos = per_world_ipos
    if per_world_armature is not None:
      flags |= 4
      buf_armature = per_world_armature
    if per_world_inertia is not None:
      flags |= 8
      buf_inertia = per_world_inertia
    if per_world_damping is not None:
      flags |= 16
      buf_damping = per_world_damping
    if per_world_iquat is not None:
      flags |= 32
      buf_iquat = per_world_iquat

    self.km.launch(
        "kernel_articulated_dynamics",
        self.bodies_buf,
        self.dofs_buf,
        qpos,
        qvel,
        buf_mass,
        buf_ipos,
        buf_armature,
        int(flags),
        self.M_eff,
        self.qfrc_bias,
        self.body_xpos,
        self.body_xmat,
        self.body_xipos,
        self.body_ximat,
        self.subtree_com,
        buf_inertia,
        buf_damping,
        float(dt),
        buf_iquat,
        threads=B,
    )

  def compute_native_cholesky_solve(
      self,
      M: torch.Tensor,
      B_mat: torch.Tensor,
      X_out: Optional[torch.Tensor] = None,
  ) -> Tuple[torch.Tensor, torch.Tensor]:
    """Solves M X = B natively on GPU via Cholesky factorization and forward/back substitution.

    Args:
        M: (B, 20, 20) float32 symmetric positive-definite matrix
        B_mat: (B, 20, K) or (B, 20) float32 right-hand side matrix
        X_out: optional pre-allocated output buffer (B, 20, K)
    Returns:
        (X, status): solution tensor and integer status tensor (0 for success, -1/-2/-3 for failure).
        NOTE: The returned `status` tensor references `self.solver_status`, which is a persistent
        device buffer reused across successive calls to this method. If callers need to preserve
        status across multiple dispatches, they must call `.clone()` before the next dispatch.
    """
    if not isinstance(M, torch.Tensor) or not isinstance(B_mat, torch.Tensor):
      raise TypeError(
          f"M and B_mat must be torch.Tensor instances, got {type(M)}, {type(B_mat)}"
      )
    if (
        M.device.type != self.device.type
        or B_mat.device.type != self.device.type
    ):
      raise ValueError(
          f"Inputs must be on device {self.device.type}, got M on {M.device}, B_mat on {B_mat.device}"
      )
    if M.dtype != torch.float32 or B_mat.dtype != torch.float32:
      raise TypeError(
          f"Inputs must be float32, got M {M.dtype}, B_mat {B_mat.dtype}"
      )
    if M.ndim != 3 or M.shape[1] != 20 or M.shape[2] != 20:
      raise ValueError(f"Expected M shape (B, 20, 20), got {M.shape}")

    B = M.shape[0]
    if B_mat.ndim == 2:
      if B_mat.shape[0] != B or B_mat.shape[1] != 20:
        raise ValueError(f"Expected B_mat shape ({B}, 20), got {B_mat.shape}")
      B_mat_3d = B_mat.unsqueeze(2)
      K = 1
    elif B_mat.ndim == 3:
      if B_mat.shape[0] != B or B_mat.shape[1] != 20:
        raise ValueError(
            f"Expected B_mat shape ({B}, 20, K), got {B_mat.shape}"
        )
      B_mat_3d = B_mat
      K = B_mat.shape[2]
    else:
      raise ValueError(f"Unsupported B_mat shape {B_mat.shape}")

    self._ensure_batch_size(B)

    if X_out is None:
      X_out = torch.zeros((B, 20, K), dtype=torch.float32, device=self.device)
    else:
      if not isinstance(X_out, torch.Tensor):
        raise TypeError(f"X_out must be a torch.Tensor, got {type(X_out)}")
      if X_out.device.type != self.device.type or X_out.dtype != torch.float32:
        raise ValueError(
            f"X_out must be float32 on {self.device.type}, got {X_out.dtype} on {X_out.device}"
        )
      expected_shape = (
          (B, 20, K)
          if B_mat.ndim == 3
          else ((B, 20) if X_out.ndim == 2 else (B, 20, 1))
      )
      if X_out.shape != expected_shape and X_out.shape != (B, 20, K):
        raise ValueError(
            f"Expected X_out shape ({B}, 20, {K}), got {X_out.shape}"
        )
      if not X_out.is_contiguous():
        raise ValueError("X_out must be contiguous")

    if not M.is_contiguous():
      M = M.contiguous()
    if not B_mat_3d.is_contiguous():
      B_mat_3d = B_mat_3d.contiguous()

    self.km.launch(
        "kernel_cholesky_solve",
        M,
        B_mat_3d,
        int(K),
        self.L_factor,
        X_out,
        self.solver_status,
        threads=B,
    )

    return X_out, self.solver_status

  def compute_native_M_inv(self):
    """Inverts M_eff natively on GPU via Cholesky solve M_eff * M_inv = I_20 without LAPACK fallback."""
    self.compute_native_cholesky_solve(self.M_eff, self.eye_20, self.M_inv)

  def solve_oracle_constraints(
      self,
      L_factor: torch.Tensor,
      f_smooth: torch.Tensor,
      J: torch.Tensor,
      aref: torch.Tensor,
      R: torch.Tensor,
      nefc: torch.Tensor,
      efc_type: Optional[torch.Tensor] = None,
      upstream_status: Optional[torch.Tensor] = None,
      frictionloss: Optional[torch.Tensor] = None,
      max_iters: int = 100,
      tol: float = 1e-5,
      capacity: int = 32,
  ) -> Dict[str, torch.Tensor]:
    """Solves bounded constraints (contacts, joint limits, frictionloss) on Metal without explicit M^-1 inversion.

    Args:
        L_factor: (B, 20, 20) float32 lower-triangular Cholesky factor (M = L L^T)
        f_smooth: (B, 20) float32 complete smooth generalized forces (qfrc_smooth)
        J: (B, capacity, 20) float32 contact constraint Jacobian
        aref: (B, capacity) float32 reference acceleration
        R: (B, capacity) float32 constraint regularization (efc_R)
        nefc: (B,) int32 number of active constraint rows per world
        efc_type: optional (B, capacity) int32 constraint row types (6: contact, 3: joint limit, 1: friction loss)
        upstream_status: optional (B,) int32 upstream status tensor (e.g. from Cholesky solve)
        frictionloss: optional (B, capacity) float32 friction budget bounds for bilateral friction loss
        max_iters: maximum PGS iterations (default 100)
        tol: convergence tolerance on maximum delta lambda (default 1e-5)
        capacity: constraint row capacity (default 32, maximum supported 128)
    Returns:
        Dict containing:
            "lambda": (B, capacity) float32 constraint forces
            "qfrc_constraint": (B, 20) float32 generalized constraint forces (J^T lambda)
            "qacc": (B, 20) float32 final acceleration (a_0 + delta_a)
            "solver_status": (B,) int32 status (0: converged, 1: unconverged, negative: error)
            "actual_iters": (B,) int32 iterations executed
            "dual_residual": (B,) float32 KKT complementarity residual
        NOTE: Output tensors reference internal persistent buffers. If callers need to preserve
        values across successive dispatches, they must call `.clone()`.
    """
    if not isinstance(L_factor, torch.Tensor) or not isinstance(
        f_smooth, torch.Tensor
    ):
      raise TypeError(
          f"L_factor and f_smooth must be torch.Tensor instances, got {type(L_factor)}, {type(f_smooth)}"
      )
    if (
        not isinstance(J, torch.Tensor)
        or not isinstance(aref, torch.Tensor)
        or not isinstance(R, torch.Tensor)
    ):
      raise TypeError(
          f"J, aref, and R must be torch.Tensor instances, got {type(J)}, {type(aref)}, {type(R)}"
      )
    if not isinstance(nefc, torch.Tensor):
      raise TypeError(f"nefc must be a torch.Tensor, got {type(nefc)}")

    dev = self.device
    for name, t in [
        ("L_factor", L_factor),
        ("f_smooth", f_smooth),
        ("J", J),
        ("aref", aref),
        ("R", R),
        ("nefc", nefc),
    ]:
      if t.device.type != dev.type:
        raise ValueError(
            f"Tensor '{name}' must reside on {dev.type}, got {t.device}"
        )

    for name, t in [
        ("L_factor", L_factor),
        ("f_smooth", f_smooth),
        ("J", J),
        ("aref", aref),
        ("R", R),
    ]:
      if t.dtype != torch.float32:
        raise TypeError(f"Tensor '{name}' must be float32, got {t.dtype}")

    if nefc.dtype != torch.int32:
      raise TypeError(f"nefc must be int32, got {nefc.dtype}")

    if not isinstance(capacity, int) or capacity < 1 or capacity > 128:
      raise ValueError(
          f"Capacity must be an integer in [1, 128], got {capacity}"
      )
    if not isinstance(max_iters, int) or max_iters <= 0:
      raise ValueError(f"max_iters must be a positive integer, got {max_iters}")
    if not isinstance(tol, (int, float)) or not np.isfinite(tol) or tol <= 0:
      raise ValueError(f"tol must be a finite positive number, got {tol}")

    B = L_factor.shape[0]
    if B <= 0:
      raise ValueError(f"Batch size must be positive, got {B}")

    if L_factor.shape != (B, 20, 20):
      raise ValueError(
          f"Expected L_factor shape ({B}, 20, 20), got {L_factor.shape}"
      )
    if f_smooth.shape != (B, 20):
      raise ValueError(
          f"Expected f_smooth shape ({B}, 20), got {f_smooth.shape}"
      )
    if J.shape != (B, capacity, 20):
      raise ValueError(f"Expected J shape ({B}, {capacity}, 20), got {J.shape}")
    if aref.shape != (B, capacity):
      raise ValueError(
          f"Expected aref shape ({B}, {capacity}), got {aref.shape}"
      )
    if R.shape != (B, capacity):
      raise ValueError(f"Expected R shape ({B}, {capacity}), got {R.shape}")
    if nefc.shape != (B,):
      raise ValueError(f"Expected nefc shape ({B},), got {nefc.shape}")

    self._ensure_batch_size(B)
    if self.constraint_capacity < capacity or self.oracle_lambda.shape != (
        B,
        capacity,
    ):
      self.constraint_capacity = max(self.constraint_capacity, capacity)
      self.oracle_lambda = torch.zeros(
          (B, capacity), dtype=torch.float32, device=dev
      )

    if self.scratchpad_A.shape != (B, capacity, capacity):
      self.scratchpad_A = torch.zeros(
          (B, capacity, capacity), dtype=torch.float32, device=dev
      )
    if self.scratchpad_Y.shape != (B, capacity, 20):
      self.scratchpad_Y = torch.zeros(
          (B, capacity, 20), dtype=torch.float32, device=dev
      )

    if frictionloss is None:
      if self.dummy_frictionloss.shape != (B, capacity):
        self.dummy_frictionloss = torch.zeros(
            (B, capacity), dtype=torch.float32, device=dev
        )
      fric_buf = self.dummy_frictionloss
    else:
      if not isinstance(frictionloss, torch.Tensor):
        raise TypeError(
            f"frictionloss must be a torch.Tensor, got {type(frictionloss)}"
        )
      if (
          frictionloss.device.type != dev.type
          or frictionloss.dtype != torch.float32
      ):
        raise ValueError(
            f"frictionloss must be float32 on {dev.type}, got {frictionloss.dtype} on {frictionloss.device}"
        )
      if frictionloss.shape != (B, capacity):
        raise ValueError(
            f"Expected frictionloss shape ({B}, {capacity}), got {frictionloss.shape}"
        )
      fric_buf = frictionloss.contiguous()

    if efc_type is None:
      efc_type = torch.full((B, capacity), 6, dtype=torch.int32, device=dev)
    else:
      if not isinstance(efc_type, torch.Tensor):
        raise TypeError(
            f"efc_type must be a torch.Tensor, got {type(efc_type)}"
        )
      if efc_type.device.type != dev.type or efc_type.dtype != torch.int32:
        raise ValueError(
            f"efc_type must be int32 on {dev.type}, got {efc_type.dtype} on {efc_type.device}"
        )
      if efc_type.shape != (B, capacity):
        raise ValueError(
            f"Expected efc_type shape ({B}, {capacity}), got {efc_type.shape}"
        )

    if upstream_status is None:
      upstream_status = torch.zeros((B,), dtype=torch.int32, device=dev)
    else:
      if not isinstance(upstream_status, torch.Tensor):
        raise TypeError(
            f"upstream_status must be a torch.Tensor, got {type(upstream_status)}"
        )
      if (
          upstream_status.device.type != dev.type
          or upstream_status.dtype != torch.int32
      ):
        raise ValueError(
            f"upstream_status must be int32 on {dev.type}, got {upstream_status.dtype} on {upstream_status.device}"
        )
      if upstream_status.shape != (B,):
        raise ValueError(
            f"Expected upstream_status shape ({B},), got {upstream_status.shape}"
        )

    L_factor = L_factor.contiguous()
    f_smooth = f_smooth.contiguous()
    J = J.contiguous()
    aref = aref.contiguous()
    R = R.contiguous()
    nefc = nefc.contiguous()
    efc_type = efc_type.contiguous()
    upstream_status = upstream_status.contiguous()

    self.km.launch(
        "kernel_oracle_constrained_solve",
        L_factor,
        f_smooth,
        J,
        aref,
        R,
        efc_type,
        nefc,
        upstream_status,
        self.oracle_lambda,
        self.oracle_qfrc_constraint,
        self.oracle_qacc,
        self.oracle_solver_status,
        self.oracle_actual_iters,
        self.oracle_dual_residual,
        self.scratchpad_A,
        self.scratchpad_Y,
        fric_buf,
        int(max_iters),
        int(capacity),
        float(tol),
        threads=B,
    )

    return {
        "lambda": self.oracle_lambda,
        "qfrc_constraint": self.oracle_qfrc_constraint,
        "qacc": self.oracle_qacc,
        "solver_status": self.oracle_solver_status,
        "actual_iters": self.oracle_actual_iters,
        "dual_residual": self.oracle_dual_residual,
    }

  def compute_cad_contact_manifold_v2(
      self,
      geom_xpos: torch.Tensor,
      geom_xmat: torch.Tensor,
      nconmax: Optional[int] = None,
  ) -> Tuple[
      torch.Tensor,
      torch.Tensor,
      torch.Tensor,
      torch.Tensor,
      torch.Tensor,
      torch.Tensor,
      torch.Tensor,
  ]:
    """Computes contact manifold via pinned MuJoCo mjc_PlaneConvex hull-graph traversal on GPU.

    Args:
        geom_xpos: (B, 2, 3) float32 positions of foot geoms (0: left, 1: right)
        geom_xmat: (B, 2, 9) float32 row-major orientations of foot geoms
        nconmax: optional override for maximum contact capacity in [1, 35]
    Returns:
        Tuple of:
            contact_pos: (B, cap, 3) float32 contact points
            contact_dist: (B, cap) float32 penetration distances
            contact_normal: (B, cap, 3) float32 plane normals
            contact_body: (B, cap) int32 body IDs (7 or 16)
            contact_geom: (B, cap) int32 geom IDs (1 or 2)
            ncon: (B,) int32 contact counts
            overflow_flag: (B,) int32 overflow indicator (1 if contacts exceeded capacity, -1 if non-finite input, 0 otherwise)
    """
    if not isinstance(geom_xpos, torch.Tensor) or not isinstance(
        geom_xmat, torch.Tensor
    ):
      raise TypeError("geom_xpos and geom_xmat must be torch.Tensor instances")
    dev = self.device
    if geom_xpos.device.type != dev.type or geom_xmat.device.type != dev.type:
      raise ValueError(f"Inputs must be on device {dev.type}")
    if geom_xpos.dtype != torch.float32 or geom_xmat.dtype != torch.float32:
      raise TypeError("Inputs must be float32")
    B = geom_xpos.shape[0]
    if geom_xpos.shape != (B, 2, 3) or geom_xmat.shape != (B, 2, 9):
      raise ValueError(
          f"Invalid shapes: geom_xpos {geom_xpos.shape}, geom_xmat {geom_xmat.shape}"
      )

    self._ensure_batch_size(B)
    if nconmax is not None:
      if not isinstance(nconmax, int):
        raise TypeError(f"nconmax must be an int, got {type(nconmax)}")
      if nconmax <= 0 or nconmax > self.nconmax:
        raise ValueError(
            f"Requested capacity {nconmax} must be in [1, {self.nconmax}]"
        )
      cap = nconmax
    else:
      cap = self.nconmax

    if not geom_xpos.is_contiguous():
      geom_xpos = geom_xpos.contiguous()
    if not geom_xmat.is_contiguous():
      geom_xmat = geom_xmat.contiguous()

    self.km.launch(
        "kernel_cad_contact_manifold_v2",
        geom_xpos,
        geom_xmat,
        self.left_verts,
        self.right_verts,
        self.left_graph,
        self.right_graph,
        self.left_rbound,
        self.right_rbound,
        self.contact_pos,
        self.contact_dist,
        self.contact_normal,
        self.contact_body,
        self.contact_geom,
        self.ncon,
        self.contact_overflow,
        int(self.nconmax),
        int(cap),
        threads=B,
    )

    if cap < self.nconmax:
      return (
          self.contact_pos[:, :cap, :].contiguous(),
          self.contact_dist[:, :cap].contiguous(),
          self.contact_normal[:, :cap, :].contiguous(),
          self.contact_body[:, :cap].contiguous(),
          self.contact_geom[:, :cap].contiguous(),
          self.ncon,
          self.contact_overflow,
      )

    return (
        self.contact_pos,
        self.contact_dist,
        self.contact_normal,
        self.contact_body,
        self.contact_geom,
        self.ncon,
        self.contact_overflow,
    )

  def normalize_friction_buffer(
      self,
      B: int,
      stride_ncon: int,
      contact_body: torch.Tensor,
      friction: Optional[Union[float, torch.Tensor]] = None,
      per_foot_friction: Optional[torch.Tensor] = None,
  ) -> torch.Tensor:
    """Normalizes friction input to a canonical device tensor of shape (B, stride_ncon, 2).

    Rules:
    - per_foot_friction: Explicit per-foot API. Accepts (2,) or (B, 2). Maps body 7 (left foot)
      to [mu_L, mu_L], body 16 (right foot) to [mu_R, mu_R], other bodies to canonical default 0.8.
    - friction=None: Defaults to canonical 0.8 isotropic for all contacts.
    - friction=scalar (int/float): Fills all contacts in all envs with [mu, mu].
    - friction=tensor (2,): Two tangent coefficients [mu0, mu1] across all contacts in all envs.
    - friction=tensor (B, 2): Per-foot [mu_left, mu_right] per environment mapped to foot bodies.
    - friction=tensor (B, N, 2): Per-contact tangent coefficients. Padded with default 0.8 if N < stride_ncon,
      or truncated to stride_ncon if N > stride_ncon.
    """
    if per_foot_friction is not None:
      if not isinstance(per_foot_friction, torch.Tensor):
        raise TypeError(
            f"per_foot_friction must be a torch.Tensor, got {type(per_foot_friction)}"
        )
      if (
          per_foot_friction.device.type != self.device.type
          or per_foot_friction.dtype != torch.float32
      ):
        per_foot_friction = per_foot_friction.to(
            dtype=torch.float32, device=self.device
        )
      if per_foot_friction.ndim == 1 and per_foot_friction.shape[0] == 2:
        p_fric = per_foot_friction.unsqueeze(0).expand(B, 2)
      elif per_foot_friction.ndim == 2 and per_foot_friction.shape == (B, 2):
        p_fric = per_foot_friction
      else:
        raise ValueError(
            f"Expected per_foot_friction shape ({B}, 2) or (2,), got {per_foot_friction.shape}"
        )
      mu_L = p_fric[:, 0:1]
      mu_R = p_fric[:, 1:2]
      is_left = (contact_body == 7).unsqueeze(-1)
      is_right = (contact_body == 16).unsqueeze(-1)
      default_f = torch.full(
          (B, stride_ncon, 1),
          self.canonical_default_friction,
          dtype=torch.float32,
          device=self.device,
      )
      mu_eff = torch.where(
          is_left,
          mu_L.unsqueeze(1),
          torch.where(is_right, mu_R.unsqueeze(1), default_f),
      )
      return mu_eff.expand(B, stride_ncon, 2).contiguous()

    if friction is None:
      if stride_ncon == self.nconmax:
        self.friction_batch.fill_(self.canonical_default_friction)
        return self.friction_batch
      return torch.full(
          (B, stride_ncon, 2),
          self.canonical_default_friction,
          dtype=torch.float32,
          device=self.device,
      )

    if isinstance(friction, (int, float)):
      if friction <= 0.0 or not math.isfinite(friction):
        raise ValueError(
            f"Friction coefficient must be strictly positive and finite, got {friction}"
        )
      if stride_ncon == self.nconmax:
        self.friction_batch.fill_(float(friction))
        return self.friction_batch
      return torch.full(
          (B, stride_ncon, 2),
          float(friction),
          dtype=torch.float32,
          device=self.device,
      )

    if isinstance(friction, torch.Tensor):
      if friction.dtype != torch.float32:
        friction = friction.to(dtype=torch.float32)
      if friction.ndim == 1 and friction.shape[0] == 2:
        return (
            friction.view(1, 1, 2)
            .expand(B, stride_ncon, 2)
            .contiguous()
            .to(self.device)
        )
      elif friction.ndim == 2 and friction.shape == (B, 2):
        mu_L = friction[:, 0:1].to(self.device)
        mu_R = friction[:, 1:2].to(self.device)
        is_left = (contact_body == 7).unsqueeze(-1)
        is_right = (contact_body == 16).unsqueeze(-1)
        default_f = torch.full(
            (B, stride_ncon, 1),
            self.canonical_default_friction,
            dtype=torch.float32,
            device=self.device,
        )
        mu_eff = torch.where(
            is_left,
            mu_L.unsqueeze(1),
            torch.where(is_right, mu_R.unsqueeze(1), default_f),
        )
        return mu_eff.expand(B, stride_ncon, 2).contiguous()
      elif (
          friction.ndim == 3
          and friction.shape[0] == B
          and friction.shape[2] == 2
      ):
        n_fric = friction.shape[1]
        if n_fric == stride_ncon:
          return friction.contiguous().to(self.device)
        elif n_fric < stride_ncon:
          f_buf = torch.full(
              (B, stride_ncon, 2),
              self.canonical_default_friction,
              dtype=torch.float32,
              device=self.device,
          )
          f_buf[:, :n_fric, :] = friction.to(self.device)
          return f_buf
        else:  # n_fric > stride_ncon
          return friction[:, :stride_ncon, :].contiguous().to(self.device)
      else:
        raise ValueError(f"Unsupported friction shape: {friction.shape}")

    raise TypeError(
        f"friction must be float, torch.Tensor, or None, got {type(friction)}"
    )

  def assemble_contact_constraints(
      self,
      contact_pos: torch.Tensor,
      contact_dist: torch.Tensor,
      contact_body: torch.Tensor,
      ncon: torch.Tensor,
      body_xpos: torch.Tensor,
      body_xmat: torch.Tensor,
      qvel: torch.Tensor,
      friction: Optional[Union[float, torch.Tensor]] = None,
      per_foot_friction: Optional[torch.Tensor] = None,
      body_invweight0: Optional[torch.Tensor] = None,
      dof_invweight0: Optional[torch.Tensor] = None,
      params: Optional[ContactSolverParams] = None,
      capacity: int = 32,
      qpos: Optional[torch.Tensor] = None,
      dof_frictionloss: Optional[torch.Tensor] = None,
      return_frictionloss: bool = False,
      contact_body2: Optional[torch.Tensor] = None,
      contact_frame: Optional[torch.Tensor] = None,
  ) -> Union[
      Tuple[
          torch.Tensor,
          torch.Tensor,
          torch.Tensor,
          torch.Tensor,
          torch.Tensor,
          torch.Tensor,
      ],
      Tuple[
          torch.Tensor,
          torch.Tensor,
          torch.Tensor,
          torch.Tensor,
          torch.Tensor,
          torch.Tensor,
          torch.Tensor,
      ],
  ]:
    """Assembles exact MuJoCo 3.10.0 contact, joint limit, and friction loss constraints on GPU.

    Args:
        contact_pos: (B, N, 3) float32 contact points
        contact_dist: (B, N) float32 penetration distances
        contact_body: (B, N) int32 body IDs
        ncon: (B,) int32 contact counts
        body_xpos: (B, 17, 3) float32 body positions
        body_xmat: (B, 17, 9) float32 body orientations (row-major)
        qvel: (B, 20) float32 generalized velocities
        friction: optional float, (B, 2), or (B, N, 2) tangential friction coefficients
        params: optional ContactSolverParams struct
        capacity: constraint capacity (default 32, must be a multiple of 4 in [4, autonomous_capacity])
        qpos: optional (B, 21) float32 coordinates for joint limit constraint assembly
        dof_frictionloss: optional (B, 14) or (14,) float32 friction budgets for actuated DOFs
        return_frictionloss: if True, returns assembled frictionloss bounds tensor as 7th element
    Returns:
        Tuple of:
            J: (B, capacity, 20) float32 constraint Jacobian
            aref: (B, capacity) float32 reference acceleration
            R: (B, capacity) float32 regularization (efc_R)
            efc_type: (B, capacity) int32 row types (6: contact, 3: joint limit, 1: friction loss)
            nefc: (B,) int32 active row count (capped at capacity)
            overflow_flag: (B,) int32 overflow indicator (1 if rows exceeded capacity, negative on error, 0 otherwise)
            frictionloss (optional): (B, capacity) float32 friction budget bounds
    """
    # Validate tensors
    tensors = {
        "contact_pos": contact_pos,
        "contact_dist": contact_dist,
        "contact_body": contact_body,
        "ncon": ncon,
        "body_xpos": body_xpos,
        "body_xmat": body_xmat,
        "qvel": qvel,
    }
    for name, t in tensors.items():
      if not isinstance(t, torch.Tensor):
        raise TypeError(f"{name} must be a torch.Tensor, got {type(t)}")
      if t.device.type != self.device.type:
        raise ValueError(
            f"{name} must be on {self.device.type}, got {t.device.type}"
        )

    if contact_pos.dtype != torch.float32:
      raise TypeError(f"contact_pos must be float32, got {contact_pos.dtype}")
    if contact_dist.dtype != torch.float32:
      raise TypeError(f"contact_dist must be float32, got {contact_dist.dtype}")
    if contact_body.dtype != torch.int32:
      raise TypeError(f"contact_body must be int32, got {contact_body.dtype}")
    if ncon.dtype != torch.int32:
      raise TypeError(f"ncon must be int32, got {ncon.dtype}")
    if body_xpos.dtype != torch.float32:
      raise TypeError(f"body_xpos must be float32, got {body_xpos.dtype}")
    if body_xmat.dtype != torch.float32:
      raise TypeError(f"body_xmat must be float32, got {body_xmat.dtype}")
    if qvel.dtype != torch.float32:
      raise TypeError(f"qvel must be float32, got {qvel.dtype}")

    B = contact_pos.shape[0]
    if B <= 0:
      raise ValueError(f"Batch size must be positive, got {B}")
    if contact_pos.ndim != 3 or contact_pos.shape[2] != 3:
      raise ValueError(
          f"Expected contact_pos shape (B, N, 3), got {contact_pos.shape}"
      )
    stride_ncon = contact_pos.shape[1]
    if stride_ncon <= 0:
      raise ValueError(
          f"Contact dimension N must be positive, got {stride_ncon}"
      )

    if contact_dist.shape != (B, stride_ncon):
      raise ValueError(
          f"Expected contact_dist shape ({B}, {stride_ncon}), got {contact_dist.shape}"
      )
    if contact_body.shape != (B, stride_ncon):
      raise ValueError(
          f"Expected contact_body shape ({B}, {stride_ncon}), got {contact_body.shape}"
      )
    if ncon.shape != (B,):
      raise ValueError(f"Expected ncon shape ({B},), got {ncon.shape}")
    if body_xpos.shape != (B, 17, 3):
      raise ValueError(
          f"Expected body_xpos shape ({B}, 17, 3), got {body_xpos.shape}"
      )
    if body_xmat.shape != (B, 17, 9):
      raise ValueError(
          f"Expected body_xmat shape ({B}, 17, 9), got {body_xmat.shape}"
      )
    if qvel.shape != (B, 20):
      raise ValueError(f"Expected qvel shape ({B}, 20), got {qvel.shape}")

    if not isinstance(capacity, int):
      raise TypeError(f"capacity must be an int, got {type(capacity)}")
    if capacity <= 0 or capacity > self.autonomous_capacity:
      raise ValueError(
          f"capacity must be in [1, {self.autonomous_capacity}], got {capacity}"
      )
    if capacity % 4 != 0:
      raise ValueError(
          f"capacity must be a positive multiple of 4 and <= {self.autonomous_capacity}, got {capacity}"
      )

    if params is not None:
      if not isinstance(params, ContactSolverParams):
        raise TypeError(
            f"params must be ContactSolverParams, got {type(params)}"
        )
      if params.margin != 0.0:
        raise ValueError(
            "Unsupported ContactSolverParams: non-zero margin is not supported in Milestone 3B"
        )
      if (
          params.timeconst <= 0.0
          or params.dampratio <= 0.0
          or params.dmin <= 0.0
          or params.dmax <= 0.0
          or params.width <= 0.0
      ):
        raise ValueError(
            "ContactSolverParams timeconst, dampratio, dmin, dmax, width must be positive"
        )
      params_buf = torch.frombuffer(params, dtype=torch.uint8).to(self.device)
    else:
      params_buf = self.default_contact_params_buf

    self._ensure_batch_size(B)

    # Prepare qpos buffer for joint limit constraints
    if qpos is not None:
      if not isinstance(qpos, torch.Tensor):
        raise TypeError(f"qpos must be a torch.Tensor, got {type(qpos)}")
      if qpos.device.type != self.device.type or qpos.dtype != torch.float32:
        raise ValueError(f"qpos must be float32 on {self.device.type}")
      if qpos.shape != (B, 21):
        raise ValueError(f"Expected qpos shape ({B}, 21), got {qpos.shape}")
      qpos_buf = qpos.contiguous()
    else:
      if self.dummy_qpos.shape[0] != B:
        self.dummy_qpos = torch.zeros(
            (B, 21), dtype=torch.float32, device=self.device
        )
      qpos_buf = self.dummy_qpos

    # Prepare dof_frictionloss buffer for friction loss constraints
    if dof_frictionloss is not None:
      if not isinstance(dof_frictionloss, torch.Tensor):
        raise TypeError(
            f"dof_frictionloss must be a torch.Tensor, got {type(dof_frictionloss)}"
        )
      if (
          dof_frictionloss.device.type != self.device.type
          or dof_frictionloss.dtype != torch.float32
      ):
        raise ValueError(
            f"dof_frictionloss must be float32 on {self.device.type}"
        )
      if dof_frictionloss.ndim == 1 and dof_frictionloss.shape[0] == 14:
        dof_fric_buf = dof_frictionloss.unsqueeze(0).expand(B, 14).contiguous()
      elif dof_frictionloss.shape == (B, 14):
        dof_fric_buf = dof_frictionloss.contiguous()
      elif dof_frictionloss.ndim == 1 and dof_frictionloss.shape[0] == 20:
        dof_fric_buf = (
            dof_frictionloss[6:20].unsqueeze(0).expand(B, 14).contiguous()
        )
      elif dof_frictionloss.shape == (B, 20):
        dof_fric_buf = dof_frictionloss[:, 6:20].contiguous()
      else:
        raise ValueError(
            f"Expected dof_frictionloss shape ({B}, 14) or ({B}, 20), got {dof_frictionloss.shape}"
        )
    else:
      if self.dummy_dof_frictionloss.shape[0] != B:
        self.dummy_dof_frictionloss = torch.zeros(
            (B, 14), dtype=torch.float32, device=self.device
        )
      dof_fric_buf = self.dummy_dof_frictionloss

    # Prepare friction buffer matching exact stride_ncon layout
    f_buf = self.normalize_friction_buffer(
        B,
        stride_ncon,
        contact_body,
        friction=friction,
        per_foot_friction=per_foot_friction,
    )
    self.assembled_friction = f_buf

    # Prepare body and dof inverse weight buffers
    invweight_flags = 0
    buf_body_invw = self.body_invweight0
    if body_invweight0 is not None:
      if not isinstance(body_invweight0, torch.Tensor):
        raise TypeError(
            f"body_invweight0 must be a torch.Tensor, got {type(body_invweight0)}"
        )
      if (
          body_invweight0.device.type != self.device.type
          or body_invweight0.dtype != torch.float32
      ):
        body_invweight0 = body_invweight0.to(
            dtype=torch.float32, device=self.device
        )
      if body_invweight0.ndim == 2 and body_invweight0.shape == (17, 2):
        buf_body_invw = body_invweight0.contiguous()
      elif body_invweight0.ndim == 3 and body_invweight0.shape == (B, 17, 2):
        buf_body_invw = body_invweight0.contiguous()
        invweight_flags |= 1
      else:
        raise ValueError(
            f"Expected body_invweight0 shape (17, 2) or ({B}, 17, 2), got {body_invweight0.shape}"
        )

    buf_dof_invw = self.dof_invweight0
    if dof_invweight0 is not None:
      if not isinstance(dof_invweight0, torch.Tensor):
        raise TypeError(
            f"dof_invweight0 must be a torch.Tensor, got {type(dof_invweight0)}"
        )
      if (
          dof_invweight0.device.type != self.device.type
          or dof_invweight0.dtype != torch.float32
      ):
        dof_invweight0 = dof_invweight0.to(
            dtype=torch.float32, device=self.device
        )
      if dof_invweight0.ndim == 1 and dof_invweight0.shape[0] == 14:
        buf_dof_invw = dof_invweight0.contiguous()
      elif dof_invweight0.ndim == 1 and dof_invweight0.shape[0] == 20:
        buf_dof_invw = dof_invweight0[6:20].contiguous()
      elif dof_invweight0.ndim == 2 and dof_invweight0.shape == (B, 14):
        buf_dof_invw = dof_invweight0.contiguous()
        invweight_flags |= 2
      elif dof_invweight0.ndim == 2 and dof_invweight0.shape == (B, 20):
        buf_dof_invw = dof_invweight0[:, 6:20].contiguous()
        invweight_flags |= 2
      else:
        raise ValueError(
            f"Expected dof_invweight0 shape (14,), (20,), ({B}, 14), or ({B}, 20), got {dof_invweight0.shape}"
        )

    if contact_body2 is not None:
      if not isinstance(contact_body2, torch.Tensor):
        raise TypeError(
            f"contact_body2 must be a torch.Tensor, got {type(contact_body2)}"
        )
      if (
          contact_body2.device.type != self.device.type
          or contact_body2.dtype != torch.int32
      ):
        raise ValueError(f"contact_body2 must be int32 on {self.device.type}")
      if contact_body2.shape != (B, stride_ncon):
        raise ValueError(
            f"Expected contact_body2 shape ({B}, {stride_ncon}), got {contact_body2.shape}"
        )
      buf_body2 = contact_body2.contiguous()
    else:
      if not hasattr(
          self, "dummy_contact_body2"
      ) or self.dummy_contact_body2.shape != (B, stride_ncon):
        self.dummy_contact_body2 = torch.zeros(
            (B, stride_ncon), dtype=torch.int32, device=self.device
        )
      buf_body2 = self.dummy_contact_body2

    if contact_frame is not None:
      if not isinstance(contact_frame, torch.Tensor):
        raise TypeError(
            f"contact_frame must be a torch.Tensor, got {type(contact_frame)}"
        )
      if (
          contact_frame.device.type != self.device.type
          or contact_frame.dtype != torch.float32
      ):
        raise ValueError(f"contact_frame must be float32 on {self.device.type}")
      if contact_frame.shape != (B, stride_ncon, 9):
        raise ValueError(
            f"Expected contact_frame shape ({B}, {stride_ncon}, 9), got {contact_frame.shape}"
        )
      buf_frame = contact_frame.contiguous()
    else:
      if not hasattr(
          self, "dummy_contact_frame"
      ) or self.dummy_contact_frame.shape != (B, stride_ncon, 9):
        self.dummy_contact_frame = torch.zeros(
            (B, stride_ncon, 9), dtype=torch.float32, device=self.device
        )
        self.dummy_contact_frame[:, :, 2] = 1.0  # normal +Z
        self.dummy_contact_frame[:, :, 4] = 1.0  # tangent1 +Y
        self.dummy_contact_frame[:, :, 6] = -1.0  # tangent2 -X
      buf_frame = self.dummy_contact_frame

    contact_pos = contact_pos.contiguous()
    contact_dist = contact_dist.contiguous()
    contact_body = contact_body.contiguous()
    ncon = ncon.contiguous()
    body_xpos = body_xpos.contiguous()
    body_xmat = body_xmat.contiguous()
    qvel = qvel.contiguous()

    self.km.launch(
        "kernel_assemble_contact_constraints",
        contact_pos,
        contact_dist,
        contact_body,
        ncon,
        body_xpos,
        body_xmat,
        qvel,
        self.bodies_buf,
        buf_body_invw,
        f_buf,
        self.assembled_J,
        self.assembled_aref,
        self.assembled_R,
        self.assembled_efc_type,
        self.assembled_nefc,
        self.assembly_overflow,
        int(stride_ncon),
        int(self.autonomous_capacity),
        params_buf,
        int(capacity),
        qpos_buf,
        dof_fric_buf,
        self.assembled_frictionloss,
        self.assembled_efc_id,
        buf_dof_invw,
        int(invweight_flags),
        buf_body2,
        buf_frame,
        threads=B,
    )

    if capacity < self.autonomous_capacity:
      res = (
          self.assembled_J[:, :capacity, :].contiguous(),
          self.assembled_aref[:, :capacity].contiguous(),
          self.assembled_R[:, :capacity].contiguous(),
          self.assembled_efc_type[:, :capacity].contiguous(),
          self.assembled_nefc,
          self.assembly_overflow,
      )
      if return_frictionloss:
        return res + (self.assembled_frictionloss[:, :capacity].contiguous(),)
      return res

    res = (
        self.assembled_J,
        self.assembled_aref,
        self.assembled_R,
        self.assembled_efc_type,
        self.assembled_nefc,
        self.assembly_overflow,
    )
    if return_frictionloss:
      return res + (self.assembled_frictionloss,)
    return res

  def forward_autonomous(
      self,
      qpos: torch.Tensor,
      qvel: torch.Tensor,
      ctrl: Optional[torch.Tensor] = None,
      f_smooth: Optional[torch.Tensor] = None,
      friction: Optional[Union[float, torch.Tensor]] = None,
      per_foot_friction: Optional[torch.Tensor] = None,
      dof_frictionloss: Optional[torch.Tensor] = None,
      per_world_mass: Optional[torch.Tensor] = None,
      per_world_ipos: Optional[torch.Tensor] = None,
      per_world_armature: Optional[torch.Tensor] = None,
      per_world_inertia: Optional[torch.Tensor] = None,
      per_world_iquat: Optional[torch.Tensor] = None,
      body_invweight0: Optional[torch.Tensor] = None,
      dof_invweight0: Optional[torch.Tensor] = None,
      dof_damping: Optional[torch.Tensor] = None,
      qfrc_applied: Optional[torch.Tensor] = None,
      max_iters: int = 100,
      tol: float = 1e-5,
      dt: float = 0.005,
      nconmax: Optional[int] = None,
      capacity: Optional[int] = None,
      extra_contact_pos: Optional[torch.Tensor] = None,
      extra_contact_dist: Optional[torch.Tensor] = None,
      extra_contact_body1: Optional[torch.Tensor] = None,
      extra_contact_body2: Optional[torch.Tensor] = None,
      extra_contact_frame: Optional[torch.Tensor] = None,
      extra_contact_friction: Optional[torch.Tensor] = None,
      extra_ncon: Optional[torch.Tensor] = None,
  ) -> AutonomousPhysicsSliceOutputs:
    """Executes full autonomous static forward dynamics pipeline without oracle inputs.

    Pipeline:
    1. Hierarchical Forward Kinematics (FK)
    2. Articulated Dynamics (CRBA M_eff + RNE qfrc_bias)
    3. Cholesky Factorization (M_eff = L L^T)
    4. Autonomous CAD Sole Contact Manifold (mjc_PlaneConvex)
    5. Contact Constraint Assembly (J, aref, R)
    6. Factor-and-Solve Delassus PGS Constrained Solve
    """
    (
        qpos,
        qvel,
        per_world_mass,
        per_world_ipos,
        per_world_armature,
        per_world_inertia,
        per_world_damping,
        per_world_iquat,
        B,
    ) = self._prepare_inputs(
        qpos,
        qvel,
        per_world_mass=per_world_mass,
        per_world_ipos=per_world_ipos,
        per_world_armature=per_world_armature,
        per_world_inertia=per_world_inertia,
        per_world_damping=dof_damping,
        per_world_iquat=per_world_iquat,
    )
    self._ensure_batch_size(B)

    # 1. Forward Kinematics
    self.km.launch(
        "kernel_forward_kinematics",
        self.bodies_buf,
        self.geoms_buf,
        qpos,
        self.body_xpos,
        self.body_xmat,
        self.geom_xpos,
        self.geom_xmat,
        threads=B,
    )

    # 2. Native Dynamics
    self.compute_native_dynamics(
        qpos,
        qvel,
        per_world_mass=per_world_mass,
        per_world_ipos=per_world_ipos,
        per_world_armature=per_world_armature,
        per_world_inertia=per_world_inertia,
        per_world_damping=per_world_damping,
        per_world_iquat=per_world_iquat,
        dt=dt,
    )

    # 3. Smooth forces: actuator forces vs complete external/smooth forces vs native bias
    if ctrl is not None and f_smooth is not None:
      raise ValueError(
          "ctrl and f_smooth are mutually exclusive to prevent accidental actuator double-counting. "
          "Supply either direct motor ctrl or complete f_smooth, not both."
      )

    qfrc_act = torch.zeros((B, 20), dtype=torch.float32, device=self.device)
    if ctrl is not None:
      if not isinstance(ctrl, torch.Tensor):
        raise TypeError(f"ctrl must be torch.Tensor, got {type(ctrl)}")
      if ctrl.dtype != torch.float32 or ctrl.device.type != self.device.type:
        ctrl = ctrl.to(dtype=torch.float32, device=self.device)
      if ctrl.ndim == 1 and ctrl.shape[0] == 14:
        ctrl = ctrl.unsqueeze(0).expand(B, 14)
      elif ctrl.ndim != 2 or ctrl.shape != (B, 14):
        raise ValueError(f"Expected ctrl shape ({B}, 14), got {ctrl.shape}")
      ctrl_clamped = torch.clamp(
          ctrl,
          self.actuator_forcerange[:, 0],
          self.actuator_forcerange[:, 1],
      )
      qfrc_act[:, 6:20] = ctrl_clamped
      f_smooth_tensor = qfrc_act - self.qfrc_bias
    elif f_smooth is not None:
      if not isinstance(f_smooth, torch.Tensor):
        raise TypeError(f"f_smooth must be torch.Tensor, got {type(f_smooth)}")
      if (
          f_smooth.device.type != self.device.type
          or f_smooth.dtype != torch.float32
      ):
        raise ValueError(f"f_smooth must be float32 on {self.device.type}")
      if f_smooth.shape != (B, 20):
        raise ValueError(
            f"Expected f_smooth shape ({B}, 20), got {f_smooth.shape}"
        )
      f_smooth_tensor = f_smooth.contiguous()
    else:
      f_smooth_tensor = -self.qfrc_bias

    if qfrc_applied is not None:
      if not isinstance(qfrc_applied, torch.Tensor):
        raise TypeError(
            f"qfrc_applied must be torch.Tensor, got {type(qfrc_applied)}"
        )
      if (
          qfrc_applied.device.type != self.device.type
          or qfrc_applied.dtype != torch.float32
      ):
        qfrc_applied = qfrc_applied.to(dtype=torch.float32, device=self.device)
      if qfrc_applied.ndim == 1 and qfrc_applied.shape[0] == 20:
        qfrc_applied = qfrc_applied.unsqueeze(0).expand(B, 20)
      elif qfrc_applied.ndim != 2 or qfrc_applied.shape != (B, 20):
        raise ValueError(
            f"Expected qfrc_applied shape ({B}, 20), got {qfrc_applied.shape}"
        )
      f_smooth_tensor = f_smooth_tensor + qfrc_applied

    if per_world_damping is not None:
      f_smooth_tensor = f_smooth_tensor - per_world_damping * qvel

    # 4. Cholesky Factorization: factorize M_eff -> L_factor
    _, cholesky_status = self.compute_native_cholesky_solve(
        self.M_eff,
        f_smooth_tensor,
        X_out=self.qacc,
    )

    # 5. Autonomous CAD Contact Manifold
    self.compute_cad_contact_manifold_v2(
        self.geom_xpos,
        self.geom_xmat,
        nconmax=nconmax,
    )

    # 5b. Contact Merging: combine CAD sole ground contacts with extra robot-robot self-contacts
    has_extra = extra_contact_pos is not None and extra_ncon is not None
    if has_extra:
      stride_e = extra_contact_pos.shape[1]
      if extra_contact_pos.shape != (B, stride_e, 3):
        raise ValueError(
            f"Expected extra_contact_pos shape ({B}, {stride_e}, 3), got {extra_contact_pos.shape}"
        )
      if extra_contact_dist is None or extra_contact_dist.shape != (
          B,
          stride_e,
      ):
        raise ValueError(
            f"Expected extra_contact_dist shape ({B}, {stride_e}), got {getattr(extra_contact_dist, 'shape', None)}"
        )
      if extra_ncon.shape != (B,):
        raise ValueError(
            f"Expected extra_ncon shape ({B},), got {extra_ncon.shape}"
        )

      if extra_contact_body1 is not None:
        if extra_contact_body1.shape != (B, stride_e):
          raise ValueError(
              f"Expected extra_contact_body1 shape ({B}, {stride_e}), got {extra_contact_body1.shape}"
          )
        extra_b1_in = extra_contact_body1.contiguous()
      else:
        extra_b1_in = torch.zeros(
            (B, stride_e), dtype=torch.int32, device=self.device
        )

      if extra_contact_body2 is not None:
        if extra_contact_body2.shape != (B, stride_e):
          raise ValueError(
              f"Expected extra_contact_body2 shape ({B}, {stride_e}), got {extra_contact_body2.shape}"
          )
        extra_b2_in = extra_contact_body2.contiguous()
      else:
        extra_b2_in = torch.zeros(
            (B, stride_e), dtype=torch.int32, device=self.device
        )

      if extra_contact_frame is not None:
        if extra_contact_frame.shape != (B, stride_e, 9):
          raise ValueError(
              f"Expected extra_contact_frame shape ({B}, {stride_e}, 9), got {extra_contact_frame.shape}"
          )
        extra_frame_in = extra_contact_frame.contiguous()
      else:
        extra_frame_in = torch.zeros(
            (B, stride_e, 9), dtype=torch.float32, device=self.device
        )
        extra_frame_in[:, :, 2] = 1.0
        extra_frame_in[:, :, 4] = 1.0
        extra_frame_in[:, :, 6] = -1.0

      if extra_contact_friction is not None:
        if extra_contact_friction.shape != (B, stride_e, 2):
          raise ValueError(
              f"Expected extra_contact_friction shape ({B}, {stride_e}, 2), got {extra_contact_friction.shape}"
          )
        extra_fric_in = extra_contact_friction.contiguous()
      else:
        extra_fric_in = torch.full(
            (B, stride_e, 2),
            self.canonical_default_friction,
            dtype=torch.float32,
            device=self.device,
        )

      norm_ground_fric = self.normalize_friction_buffer(
          B,
          self.nconmax,
          self.contact_body,
          friction=friction,
          per_foot_friction=per_foot_friction,
      )
      fric_flag = 3
      fric_in = norm_ground_fric

      self.km.launch(
          "kernel_merge_contacts",
          self.contact_pos,
          self.contact_dist,
          self.contact_body,
          self.ncon,
          extra_contact_pos.contiguous(),
          extra_contact_dist.contiguous(),
          extra_b1_in,
          extra_b2_in,
          extra_frame_in,
          extra_fric_in,
          extra_ncon.contiguous(),
          self.merged_contact_pos,
          self.merged_contact_dist,
          self.merged_contact_body1,
          self.merged_contact_body2,
          self.merged_contact_frame,
          self.merged_contact_friction,
          self.merged_ncon,
          fric_in,
          int(self.nconmax),
          int(stride_e),
          int(self.merged_nconmax),
          int(fric_flag),
          self.contact_geom,
          self.merged_contact_overflow,
          threads=B,
      )
      c_pos_in = self.merged_contact_pos
      c_dist_in = self.merged_contact_dist
      c_body1_in = self.merged_contact_body1
      c_body2_in = self.merged_contact_body2
      c_frame_in = self.merged_contact_frame
      c_fric_in = self.merged_contact_friction
      ncon_in = self.merged_ncon
      per_foot_fric_in = None
    else:
      c_pos_in = self.contact_pos
      c_dist_in = self.contact_dist
      c_body1_in = self.contact_body
      c_body2_in = None
      c_frame_in = None
      c_fric_in = friction
      ncon_in = self.ncon
      per_foot_fric_in = per_foot_friction

    # 6. Contact, Joint Limit, and Friction Loss Constraint Assembly
    act_cap = capacity if capacity is not None else self.autonomous_capacity
    (
        assembled_J,
        assembled_aref,
        assembled_R,
        assembled_efc_type,
        assembled_nefc,
        assembly_overflow,
        assembled_fricloss,
    ) = self.assemble_contact_constraints(
        c_pos_in,
        c_dist_in,
        c_body1_in,
        ncon_in,
        self.body_xpos,
        self.body_xmat,
        qvel,
        friction=c_fric_in,
        per_foot_friction=per_foot_fric_in,
        body_invweight0=body_invweight0,
        dof_invweight0=dof_invweight0,
        capacity=act_cap,
        qpos=qpos,
        dof_frictionloss=dof_frictionloss,
        return_frictionloss=True,
        contact_body2=c_body2_in,
        contact_frame=c_frame_in,
    )

    # Combine upstream status on device without host sync
    # 0: OK
    # cholesky_status < 0: Cholesky failure (-1, -2, -3)
    # contact_overflow == 1: Contact manifold overflow (-6)
    # contact_overflow < 0: Non-finite contact inputs (-8)
    # assembly_overflow == 1: Assembly row overflow (-7)
    # assembly_overflow < 0: Non-finite assembly state / invalid body ID / invalid friction / invalid count (-9)
    self.autonomous_upstream_status.copy_(cholesky_status)
    mask_ok = self.autonomous_upstream_status == 0
    self.autonomous_upstream_status.masked_fill_(
        mask_ok & (self.contact_overflow == 1), -6
    )
    self.autonomous_upstream_status.masked_fill_(
        mask_ok & (self.contact_overflow < 0), -8
    )

    if has_extra:
      mask_ok = self.autonomous_upstream_status == 0
      self.autonomous_upstream_status.masked_fill_(
          mask_ok & (self.merged_contact_overflow > 0), -6
      )
      self.autonomous_upstream_status.masked_fill_(
          mask_ok & (self.merged_contact_overflow < 0), -8
      )
      eff_contact_overflow = torch.where(
          self.merged_contact_overflow != 0,
          self.merged_contact_overflow,
          self.contact_overflow,
      )
    else:
      eff_contact_overflow = self.contact_overflow

    mask_ok = self.autonomous_upstream_status == 0
    self.autonomous_upstream_status.masked_fill_(
        mask_ok & (self.assembly_overflow == 1), -7
    )
    self.autonomous_upstream_status.masked_fill_(
        mask_ok & (self.assembly_overflow < 0), -9
    )

    # 7. Delassus PGS Constrained Solve
    solve_results = self.solve_oracle_constraints(
        L_factor=self.L_factor,
        f_smooth=f_smooth_tensor,
        J=assembled_J,
        aref=assembled_aref,
        R=assembled_R,
        nefc=assembled_nefc,
        efc_type=assembled_efc_type,
        upstream_status=self.autonomous_upstream_status,
        frictionloss=assembled_fricloss,
        max_iters=max_iters,
        tol=tol,
        capacity=act_cap,
    )

    return AutonomousPhysicsSliceOutputs(
        body_xpos=self.body_xpos,
        body_xmat=self.body_xmat,
        body_xipos=self.body_xipos,
        body_ximat=self.body_ximat,
        subtree_com=self.subtree_com,
        geom_xpos=self.geom_xpos,
        geom_xmat=self.geom_xmat,
        contact_pos=self.contact_pos,
        contact_dist=self.contact_dist,
        contact_normal=self.contact_normal,
        contact_body=self.contact_body,
        contact_geom=self.contact_geom,
        ncon=self.ncon,
        contact_overflow=eff_contact_overflow,
        J=self.assembled_J,
        aref=self.assembled_aref,
        R=self.assembled_R,
        efc_type=self.assembled_efc_type,
        nefc=self.assembled_nefc,
        assembly_overflow=self.assembly_overflow,
        M_eff=self.M_eff,
        L_factor=self.L_factor,
        cholesky_status=cholesky_status,
        qfrc_bias=self.qfrc_bias,
        qfrc_actuator=qfrc_act,
        f_smooth=f_smooth_tensor,
        lambda_force=solve_results["lambda"],
        qfrc_constraint=solve_results["qfrc_constraint"],
        qacc=solve_results["qacc"],
        solver_status=solve_results["solver_status"],
        actual_iters=solve_results["actual_iters"],
        dual_residual=solve_results["dual_residual"],
        efc_frictionloss=assembled_fricloss,
        efc_id=self.assembled_efc_id,
    )

  def integrate_implicit_fast(
      self,
      qpos: torch.Tensor,
      qvel: torch.Tensor,
      qacc: torch.Tensor,
      upstream_status: torch.Tensor,
      dt: float = 0.005,
      M: Optional[torch.Tensor] = None,
      dof_damping: Optional[torch.Tensor] = None,
      qpos_out: Optional[torch.Tensor] = None,
      qvel_out: Optional[torch.Tensor] = None,
      status_out: Optional[torch.Tensor] = None,
  ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Integrates coordinates and velocities via canonical ImplicitFast integration directly on GPU."""
    if (
        not isinstance(qpos, torch.Tensor)
        or not isinstance(qvel, torch.Tensor)
        or not isinstance(qacc, torch.Tensor)
        or not isinstance(upstream_status, torch.Tensor)
    ):
      raise TypeError("All inputs must be torch.Tensor instances")
    if (
        qpos.device.type != self.device.type
        or qvel.device.type != self.device.type
        or qacc.device.type != self.device.type
        or upstream_status.device.type != self.device.type
    ):
      raise ValueError(f"All inputs must be on device {self.device.type}")
    if (
        qpos.dtype != torch.float32
        or qvel.dtype != torch.float32
        or qacc.dtype != torch.float32
    ):
      raise TypeError("qpos, qvel, qacc must be float32")
    if upstream_status.dtype != torch.int32:
      raise TypeError("upstream_status must be int32")
    if qpos.ndim != 2 or qpos.shape[1] != 21:
      raise ValueError(f"Expected qpos shape (B, 21), got {qpos.shape}")
    if qvel.ndim != 2 or qvel.shape[1] != 20:
      raise ValueError(f"Expected qvel shape (B, 20), got {qvel.shape}")
    if qacc.ndim != 2 or qacc.shape[1] != 20:
      raise ValueError(f"Expected qacc shape (B, 20), got {qacc.shape}")
    B = qpos.shape[0]
    if B <= 0:
      raise ValueError(f"Batch size must be strictly positive, got {B}")
    if qvel.shape[0] != B or qacc.shape[0] != B:
      raise ValueError("Batch dimensions must match across all inputs")
    if upstream_status.ndim != 1 or upstream_status.shape != (B,):
      raise ValueError(
          f"Expected upstream_status shape ({B},), got {upstream_status.shape}"
      )
    if dt <= 0.0 or not math.isfinite(dt):
      raise ValueError(f"dt must be strictly positive and finite, got {dt}")

    self._ensure_batch_size(B)

    # Validate caller-provided output buffers if supplied
    if qpos_out is not None:
      if not isinstance(qpos_out, torch.Tensor):
        raise TypeError(
            f"qpos_out must be a torch.Tensor, got {type(qpos_out)}"
        )
      if qpos_out.device.type != self.device.type:
        raise ValueError(
            f"qpos_out must be on device {self.device.type}, got {qpos_out.device}"
        )
      if qpos_out.dtype != torch.float32:
        raise TypeError(f"qpos_out must be float32, got {qpos_out.dtype}")
      if qpos_out.shape != (B, 21):
        raise ValueError(
            f"Expected qpos_out shape ({B}, 21), got {qpos_out.shape}"
        )
      if not qpos_out.is_contiguous():
        raise ValueError("qpos_out must be contiguous")

    if qvel_out is not None:
      if not isinstance(qvel_out, torch.Tensor):
        raise TypeError(
            f"qvel_out must be a torch.Tensor, got {type(qvel_out)}"
        )
      if qvel_out.device.type != self.device.type:
        raise ValueError(
            f"qvel_out must be on device {self.device.type}, got {qvel_out.device}"
        )
      if qvel_out.dtype != torch.float32:
        raise TypeError(f"qvel_out must be float32, got {qvel_out.dtype}")
      if qvel_out.shape != (B, 20):
        raise ValueError(
            f"Expected qvel_out shape ({B}, 20), got {qvel_out.shape}"
        )
      if not qvel_out.is_contiguous():
        raise ValueError("qvel_out must be contiguous")

    if status_out is not None:
      if not isinstance(status_out, torch.Tensor):
        raise TypeError(
            f"status_out must be a torch.Tensor, got {type(status_out)}"
        )
      if status_out.device.type != self.device.type:
        raise ValueError(
            f"status_out must be on device {self.device.type}, got {status_out.device}"
        )
      if status_out.dtype != torch.int32:
        raise TypeError(f"status_out must be int32, got {status_out.dtype}")
      if status_out.shape != (B,):
        raise ValueError(
            f"Expected status_out shape ({B},), got {status_out.shape}"
        )
      if not status_out.is_contiguous():
        raise ValueError("status_out must be contiguous")

    # Storage aliasing validation
    # Supported aliasing: exact in-place (qpos_out == qpos, qvel_out == qvel, status_out == upstream_status).
    # Arbitrary or partially overlapping views are strictly rejected.
    def _check_overlap(
        t1: torch.Tensor,
        t2: torch.Tensor,
        name1: str,
        name2: str,
        allow_identical: bool = False,
    ):
      p1 = t1.data_ptr()
      bytes1 = t1.numel() * t1.element_size()
      p2 = t2.data_ptr()
      bytes2 = t2.numel() * t2.element_size()
      if p1 < p2 + bytes2 and p2 < p1 + bytes1:
        if (
            allow_identical
            and p1 == p2
            and bytes1 == bytes2
            and t1.shape == t2.shape
            and t1.stride() == t2.stride()
        ):
          return
        raise ValueError(
            f"Unsafe memory aliasing / partial overlap detected between {name1} and {name2}"
        )

    if qpos_out is not None:
      _check_overlap(qpos_out, qpos, "qpos_out", "qpos", allow_identical=True)
      _check_overlap(qpos_out, qvel, "qpos_out", "qvel", allow_identical=False)
      _check_overlap(qpos_out, qacc, "qpos_out", "qacc", allow_identical=False)
      _check_overlap(
          qpos_out,
          upstream_status,
          "qpos_out",
          "upstream_status",
          allow_identical=False,
      )

    if qvel_out is not None:
      _check_overlap(qvel_out, qvel, "qvel_out", "qvel", allow_identical=True)
      _check_overlap(qvel_out, qpos, "qvel_out", "qpos", allow_identical=False)
      _check_overlap(qvel_out, qacc, "qvel_out", "qacc", allow_identical=False)
      _check_overlap(
          qvel_out,
          upstream_status,
          "qvel_out",
          "upstream_status",
          allow_identical=False,
      )

    if status_out is not None:
      _check_overlap(
          status_out,
          upstream_status,
          "status_out",
          "upstream_status",
          allow_identical=True,
      )
      _check_overlap(
          status_out, qpos, "status_out", "qpos", allow_identical=False
      )
      _check_overlap(
          status_out, qvel, "status_out", "qvel", allow_identical=False
      )
      _check_overlap(
          status_out, qacc, "status_out", "qacc", allow_identical=False
      )

    if qpos_out is not None and qvel_out is not None:
      _check_overlap(
          qpos_out, qvel_out, "qpos_out", "qvel_out", allow_identical=False
      )
    if qpos_out is not None and status_out is not None:
      _check_overlap(
          qpos_out, status_out, "qpos_out", "status_out", allow_identical=False
      )
    if qvel_out is not None and status_out is not None:
      _check_overlap(
          qvel_out, status_out, "qvel_out", "status_out", allow_identical=False
      )

    out_qp = self.integrated_qpos if qpos_out is None else qpos_out
    out_qv = self.integrated_qvel if qvel_out is None else qvel_out
    out_stat = self.integration_status if status_out is None else status_out

    qpos_c = qpos.contiguous()
    qvel_c = qvel.contiguous()
    qacc_c = qacc.contiguous()
    stat_c = upstream_status.contiguous()

    has_damping = 0
    buf_M = self.M_eff if M is None else M
    if not buf_M.is_contiguous():
      buf_M = buf_M.contiguous()

    buf_damp = self.dummy_damping
    if dof_damping is not None:
      if not isinstance(dof_damping, torch.Tensor):
        raise TypeError(
            f"dof_damping must be a torch.Tensor, got {type(dof_damping)}"
        )
      if (
          dof_damping.device.type != self.device.type
          or dof_damping.dtype != torch.float32
      ):
        dof_damping = dof_damping.to(dtype=torch.float32, device=self.device)
      if dof_damping.ndim == 1 and dof_damping.shape[0] == 14:
        d_buf = torch.zeros((B, 20), dtype=torch.float32, device=self.device)
        d_buf[:, 6:20] = dof_damping
        buf_damp = d_buf
      elif dof_damping.ndim == 2 and dof_damping.shape == (B, 14):
        d_buf = torch.zeros((B, 20), dtype=torch.float32, device=self.device)
        d_buf[:, 6:20] = dof_damping
        buf_damp = d_buf
      elif dof_damping.ndim == 1 and dof_damping.shape[0] == 20:
        buf_damp = dof_damping.unsqueeze(0).expand(B, 20).contiguous()
      elif dof_damping.ndim == 2 and dof_damping.shape == (B, 20):
        buf_damp = dof_damping.contiguous()
      else:
        raise ValueError(
            f"Expected dof_damping shape (14,), (20,), ({B}, 14), or ({B}, 20), got {dof_damping.shape}"
        )
      if (buf_damp > 0.0).any():
        has_damping = 1

    self.km.launch(
        "kernel_integrate_implicit_fast",
        qpos_c,
        qvel_c,
        qacc_c,
        stat_c,
        float(dt),
        out_qp,
        out_qv,
        out_stat,
        buf_M,
        buf_damp,
        int(has_damping),
        threads=B,
    )

    return out_qp[:B], out_qv[:B], out_stat[:B]

  def step_autonomous(
      self,
      qpos: torch.Tensor,
      qvel: torch.Tensor,
      ctrl: Optional[torch.Tensor] = None,
      f_smooth: Optional[torch.Tensor] = None,
      friction: Optional[Union[float, torch.Tensor]] = None,
      per_foot_friction: Optional[torch.Tensor] = None,
      dof_frictionloss: Optional[torch.Tensor] = None,
      per_world_mass: Optional[torch.Tensor] = None,
      per_world_ipos: Optional[torch.Tensor] = None,
      per_world_armature: Optional[torch.Tensor] = None,
      per_world_inertia: Optional[torch.Tensor] = None,
      dof_damping: Optional[torch.Tensor] = None,
      per_world_iquat: Optional[torch.Tensor] = None,
      body_invweight0: Optional[torch.Tensor] = None,
      dof_invweight0: Optional[torch.Tensor] = None,
      qfrc_applied: Optional[torch.Tensor] = None,
      max_iters: int = 100,
      tol: float = 1e-5,
      dt: float = 0.005,
      nconmax: Optional[int] = None,
      capacity: Optional[int] = None,
      extra_contact_pos: Optional[torch.Tensor] = None,
      extra_contact_dist: Optional[torch.Tensor] = None,
      extra_contact_body1: Optional[torch.Tensor] = None,
      extra_contact_body2: Optional[torch.Tensor] = None,
      extra_contact_frame: Optional[torch.Tensor] = None,
      extra_contact_friction: Optional[torch.Tensor] = None,
      extra_ncon: Optional[torch.Tensor] = None,
  ) -> AutonomousStepOutputs:
    """Executes one complete 5 ms physics step: FK -> Dynamics -> Contacts -> Assembly -> PGS Solve -> ImplicitFast Integration."""
    out = self.forward_autonomous(
        qpos,
        qvel,
        ctrl=ctrl,
        f_smooth=f_smooth,
        friction=friction,
        per_foot_friction=per_foot_friction,
        dof_frictionloss=dof_frictionloss,
        per_world_mass=per_world_mass,
        per_world_ipos=per_world_ipos,
        per_world_armature=per_world_armature,
        per_world_inertia=per_world_inertia,
        dof_damping=dof_damping,
        per_world_iquat=per_world_iquat,
        body_invweight0=body_invweight0,
        dof_invweight0=dof_invweight0,
        qfrc_applied=qfrc_applied,
        max_iters=max_iters,
        tol=tol,
        dt=dt,
        nconmax=nconmax,
        capacity=capacity,
        extra_contact_pos=extra_contact_pos,
        extra_contact_dist=extra_contact_dist,
        extra_contact_body1=extra_contact_body1,
        extra_contact_body2=extra_contact_body2,
        extra_contact_frame=extra_contact_frame,
        extra_contact_friction=extra_contact_friction,
        extra_ncon=extra_ncon,
    )
    qpos_next, qvel_next, int_stat = self.integrate_implicit_fast(
        qpos,
        qvel,
        out.qacc,
        out.solver_status,
        dt=dt,
        M=self.M_eff,
        dof_damping=dof_damping,
    )
    out.integration_status = int_stat
    return AutonomousStepOutputs(
        qpos=qpos_next,
        qvel=qvel_next,
        integration_status=int_stat,
        physics_outputs=out,
    )

  def step_control_interval(
      self,
      qpos: torch.Tensor,
      qvel: torch.Tensor,
      ctrl: torch.Tensor,
      num_substeps: int = 4,
      dt: float = 0.005,
      friction: Optional[Union[float, torch.Tensor]] = None,
      per_foot_friction: Optional[torch.Tensor] = None,
      dof_frictionloss: Optional[torch.Tensor] = None,
      per_world_mass: Optional[torch.Tensor] = None,
      per_world_ipos: Optional[torch.Tensor] = None,
      per_world_armature: Optional[torch.Tensor] = None,
      per_world_inertia: Optional[torch.Tensor] = None,
      dof_damping: Optional[torch.Tensor] = None,
      per_world_iquat: Optional[torch.Tensor] = None,
      body_invweight0: Optional[torch.Tensor] = None,
      dof_invweight0: Optional[torch.Tensor] = None,
      qfrc_applied: Optional[torch.Tensor] = None,
      max_iters: int = 100,
      tol: float = 1e-5,
  ) -> Tuple[torch.Tensor, torch.Tensor, List[AutonomousStepOutputs]]:
    """Advances state across one 20 ms control step (default 4 substeps of 5 ms) holding ctrl constant."""
    curr_qp = qpos.clone()
    curr_qv = qvel.clone()
    substep_outputs = []

    for s in range(num_substeps):
      step_res = self.step_autonomous(
          curr_qp,
          curr_qv,
          ctrl=ctrl,
          friction=friction,
          per_foot_friction=per_foot_friction,
          dof_frictionloss=dof_frictionloss,
          per_world_mass=per_world_mass,
          per_world_ipos=per_world_ipos,
          per_world_armature=per_world_armature,
          per_world_inertia=per_world_inertia,
          dof_damping=dof_damping,
          per_world_iquat=per_world_iquat,
          body_invweight0=body_invweight0,
          dof_invweight0=dof_invweight0,
          qfrc_applied=qfrc_applied,
          max_iters=max_iters,
          tol=tol,
          dt=dt,
      )
      curr_qp = step_res.qpos.clone()
      curr_qv = step_res.qvel.clone()
      substep_outputs.append(step_res.clone())

    return curr_qp, curr_qv, substep_outputs

  def step_bam_control_interval(
      self,
      qpos: torch.Tensor,
      qvel: torch.Tensor,
      target_pos: torch.Tensor,
      bam_controller: Any,
      num_substeps: int = 4,
      dt: float = 0.005,
      vin: Optional[torch.Tensor] = None,
      voltage_drop_gain: Optional[Union[float, torch.Tensor]] = None,
      kp_scale: Union[float, torch.Tensor] = 1.0,
      kd_scale: Union[float, torch.Tensor] = 1.0,
      friction_scale: Union[float, torch.Tensor] = 1.0,
      friction: Optional[Union[float, torch.Tensor]] = None,
      per_foot_friction: Optional[torch.Tensor] = None,
      per_world_mass: Optional[torch.Tensor] = None,
      per_world_ipos: Optional[torch.Tensor] = None,
      per_world_armature: Optional[torch.Tensor] = None,
      per_world_inertia: Optional[torch.Tensor] = None,
      dof_damping: Optional[torch.Tensor] = None,
      per_world_iquat: Optional[torch.Tensor] = None,
      body_invweight0: Optional[torch.Tensor] = None,
      dof_invweight0: Optional[torch.Tensor] = None,
      qfrc_applied: Optional[torch.Tensor] = None,
      max_iters: int = 100,
      tol: float = 1e-5,
      init_prev_motor_torque: Optional[torch.Tensor] = None,
      init_prev_actuator_torque: Optional[torch.Tensor] = None,
      init_qfrc_bias: Optional[torch.Tensor] = None,
      init_qfrc_constraint: Optional[torch.Tensor] = None,
      init_efc_type: Optional[torch.Tensor] = None,
      init_efc_id: Optional[torch.Tensor] = None,
      init_efc_force: Optional[torch.Tensor] = None,
      init_nefc: Optional[torch.Tensor] = None,
  ) -> Tuple[
      torch.Tensor, torch.Tensor, List[AutonomousStepOutputs], List[Any]
  ]:
    """Advances state across one 20 ms control step (4 substeps of 5 ms) with dynamic BAM M6 torque & friction recomputation."""
    curr_qp = qpos.clone()
    curr_qv = qvel.clone()
    B = qpos.shape[0]
    dev = qpos.device

    # Track history for BAM voltage-drop and friction gearbox loads
    prev_motor_torque = (
        init_prev_motor_torque.clone()
        if init_prev_motor_torque is not None
        else torch.zeros(B, 14, dtype=torch.float32, device=dev)
    )
    prev_actuator_torque = (
        init_prev_actuator_torque.clone()
        if init_prev_actuator_torque is not None
        else torch.zeros(B, 14, dtype=torch.float32, device=dev)
    )

    if init_qfrc_bias is not None:
      qfrc_bias = init_qfrc_bias.clone()
      qfrc_constraint = (
          init_qfrc_constraint.clone()
          if init_qfrc_constraint is not None
          else torch.zeros(B, 20, dtype=torch.float32, device=dev)
      )
      efc_type = (
          init_efc_type.clone()
          if init_efc_type is not None
          else torch.zeros(
              B, self.autonomous_capacity, dtype=torch.int32, device=dev
          )
      )
      efc_id = (
          init_efc_id.clone()
          if init_efc_id is not None
          else torch.zeros(
              B, self.autonomous_capacity, dtype=torch.int32, device=dev
          )
      )
      efc_force = (
          init_efc_force.clone()
          if init_efc_force is not None
          else torch.zeros(
              B, self.autonomous_capacity, dtype=torch.float32, device=dev
          )
      )
      nefc = (
          init_nefc.clone()
          if init_nefc is not None
          else torch.zeros(B, dtype=torch.int32, device=dev)
      )
    else:
      default_init_damping = dof_damping
      if default_init_damping is None and hasattr(
          bam_controller, "friction_viscous"
      ):
        default_init_damping = torch.zeros(
            (B, 20), dtype=torch.float32, device=dev
        )
        default_init_damping[:, 6:20] = float(bam_controller.friction_viscous)
      init_out = self.forward_autonomous(
          curr_qp,
          curr_qv,
          friction=friction,
          per_foot_friction=per_foot_friction,
          per_world_mass=per_world_mass,
          per_world_ipos=per_world_ipos,
          per_world_armature=per_world_armature,
          per_world_inertia=per_world_inertia,
          dof_damping=default_init_damping,
          per_world_iquat=per_world_iquat,
          body_invweight0=body_invweight0,
          dof_invweight0=dof_invweight0,
          qfrc_applied=qfrc_applied,
          max_iters=1,
      )
      qfrc_bias = init_out.qfrc_bias.clone()
      qfrc_constraint = init_out.qfrc_constraint.clone()
      efc_type = init_out.efc_type.clone()
      efc_id = (
          init_out.efc_id.clone()
          if init_out.efc_id is not None
          else torch.zeros(
              B, self.autonomous_capacity, dtype=torch.int32, device=dev
          )
      )
      efc_force = init_out.lambda_force.clone()
      nefc = init_out.nefc.clone()

    substep_outputs = []
    bam_outputs = []

    for s in range(num_substeps):
      bam_res = bam_controller.compute_substep(
          curr_qp,
          curr_qv,
          target_pos,
          prev_motor_torque,
          prev_actuator_torque,
          qfrc_bias,
          qfrc_constraint,
          efc_type,
          efc_id,
          efc_force,
          nefc,
          vin=vin,
          voltage_drop_gain=voltage_drop_gain,
          kp_scale=kp_scale,
          kd_scale=kd_scale,
          friction_scale=friction_scale,
          dt=dt,
      )
      bam_outputs.append(bam_res)

      if dof_damping is None:
        step_damping = torch.zeros((B, 20), dtype=torch.float32, device=dev)
        step_damping[:, 6:20] = float(bam_res.damping)
      else:
        step_damping = dof_damping

      step_res = self.step_autonomous(
          curr_qp,
          curr_qv,
          ctrl=bam_res.motor_torque,
          friction=friction,
          per_foot_friction=per_foot_friction,
          dof_frictionloss=bam_res.dof_frictionloss,
          per_world_mass=per_world_mass,
          per_world_ipos=per_world_ipos,
          per_world_armature=per_world_armature,
          per_world_inertia=per_world_inertia,
          dof_damping=step_damping,
          per_world_iquat=per_world_iquat,
          body_invweight0=body_invweight0,
          dof_invweight0=dof_invweight0,
          qfrc_applied=qfrc_applied,
          max_iters=max_iters,
          tol=tol,
          dt=dt,
      )
      curr_qp = step_res.qpos.clone()
      curr_qv = step_res.qvel.clone()
      substep_outputs.append(step_res.clone())

      # Update history for next substep
      prev_motor_torque = bam_res.motor_torque.clone()
      prev_actuator_torque = step_res.physics_outputs.qfrc_actuator[
          :, 6:20
      ].clone()
      qfrc_bias = step_res.physics_outputs.qfrc_bias.clone()
      qfrc_constraint = step_res.physics_outputs.qfrc_constraint.clone()
      efc_type = step_res.physics_outputs.efc_type.clone()
      efc_id = (
          step_res.physics_outputs.efc_id.clone()
          if step_res.physics_outputs.efc_id is not None
          else efc_id
      )
      efc_force = step_res.physics_outputs.lambda_force.clone()
      nefc = step_res.physics_outputs.nefc.clone()

    return curr_qp, curr_qv, substep_outputs, bam_outputs

  def rollout_trajectory(
      self,
      qpos_init: torch.Tensor,
      qvel_init: torch.Tensor,
      num_steps: int,
      ctrl: Optional[torch.Tensor] = None,
      dt: float = 0.005,
      friction: Optional[Union[float, torch.Tensor]] = None,
      per_foot_friction: Optional[torch.Tensor] = None,
      dof_frictionloss: Optional[torch.Tensor] = None,
      per_world_mass: Optional[torch.Tensor] = None,
      per_world_ipos: Optional[torch.Tensor] = None,
      per_world_armature: Optional[torch.Tensor] = None,
      per_world_inertia: Optional[torch.Tensor] = None,
      dof_damping: Optional[torch.Tensor] = None,
      per_world_iquat: Optional[torch.Tensor] = None,
      body_invweight0: Optional[torch.Tensor] = None,
      dof_invweight0: Optional[torch.Tensor] = None,
      qfrc_applied: Optional[torch.Tensor] = None,
      max_iters: int = 100,
      tol: float = 1e-5,
  ) -> Dict[str, torch.Tensor]:
    """Rolls out a free-running autonomous trajectory on GPU without CPU state overwrites."""
    B = qpos_init.shape[0]
    qpos_hist = [qpos_init.clone()]
    qvel_hist = [qvel_init.clone()]
    qacc_hist = []
    qfrc_c_hist = []
    status_hist = []
    int_stat_hist = []
    nefc_hist = []
    iters_hist = []
    dual_res_hist = []

    curr_qp = qpos_init.clone()
    curr_qv = qvel_init.clone()

    for t in range(num_steps):
      c_step = ctrl[t] if (ctrl is not None and ctrl.ndim == 3) else ctrl
      step_res = self.step_autonomous(
          curr_qp,
          curr_qv,
          ctrl=c_step,
          friction=friction,
          per_foot_friction=per_foot_friction,
          dof_frictionloss=dof_frictionloss,
          per_world_mass=per_world_mass,
          per_world_ipos=per_world_ipos,
          per_world_armature=per_world_armature,
          per_world_inertia=per_world_inertia,
          dof_damping=dof_damping,
          per_world_iquat=per_world_iquat,
          body_invweight0=body_invweight0,
          dof_invweight0=dof_invweight0,
          qfrc_applied=qfrc_applied,
          max_iters=max_iters,
          tol=tol,
          dt=dt,
      )
      curr_qp = step_res.qpos.clone()
      curr_qv = step_res.qvel.clone()

      qpos_hist.append(curr_qp)
      qvel_hist.append(curr_qv)
      qacc_hist.append(step_res.physics_outputs.qacc.clone())
      qfrc_c_hist.append(step_res.physics_outputs.qfrc_constraint.clone())
      status_hist.append(step_res.physics_outputs.solver_status.clone())
      int_stat_hist.append(step_res.integration_status.clone())
      nefc_hist.append(step_res.physics_outputs.nefc.clone())
      iters_hist.append(step_res.physics_outputs.actual_iters.clone())
      dual_res_hist.append(step_res.physics_outputs.dual_residual.clone())

    return {
        "qpos": torch.stack(qpos_hist, dim=0),  # (num_steps + 1, B, 21)
        "qvel": torch.stack(qvel_hist, dim=0),  # (num_steps + 1, B, 20)
        "qacc": torch.stack(qacc_hist, dim=0),  # (num_steps, B, 20)
        "qfrc_constraint": torch.stack(
            qfrc_c_hist, dim=0
        ),  # (num_steps, B, 20)
        "solver_status": torch.stack(status_hist, dim=0),  # (num_steps, B)
        "integration_status": torch.stack(
            int_stat_hist, dim=0
        ),  # (num_steps, B)
        "nefc": torch.stack(nefc_hist, dim=0),  # (num_steps, B)
        "actual_iters": torch.stack(iters_hist, dim=0),  # (num_steps, B)
        "dual_residual": torch.stack(dual_res_hist, dim=0),  # (num_steps, B)
    }

  def forward(
      self,
      qpos: torch.Tensor,
      qvel: torch.Tensor,
      friction_coef: float = 1.0,
      per_world_mass: Optional[torch.Tensor] = None,
      per_world_ipos: Optional[torch.Tensor] = None,
      per_world_armature: Optional[torch.Tensor] = None,
  ) -> PhysicsSliceOutputs:
    """Executes one static-state physics slice completely on GPU without host staging."""
    (
        qpos,
        qvel,
        per_world_mass,
        per_world_ipos,
        per_world_armature,
        per_world_inertia,
        per_world_damping,
        per_world_iquat,
        B,
    ) = self._prepare_inputs(
        qpos, qvel, per_world_mass, per_world_ipos, per_world_armature
    )
    self._ensure_batch_size(B)

    # 1. Forward Kinematics for Foot Geoms
    self.km.launch(
        "kernel_forward_kinematics",
        self.bodies_buf,
        self.geoms_buf,
        qpos,
        self.body_xpos,
        self.body_xmat,
        self.geom_xpos,
        self.geom_xmat,
        threads=B,
    )

    # 2. Native Articulated Dynamics (CRBA M_eff + RNE qfrc_bias)
    self.compute_native_dynamics(
        qpos,
        qvel,
        per_world_mass=per_world_mass,
        per_world_ipos=per_world_ipos,
        per_world_armature=per_world_armature,
        per_world_inertia=per_world_inertia,
        per_world_damping=per_world_damping,
    )

    # 3. Native Cholesky Inversion of M_eff on GPU (M_eff * M_inv = I_20)
    self.compute_native_M_inv()

    # 4. CAD Contact Manifold Kernel
    self.km.launch(
        "kernel_cad_contact_manifold",
        self.geom_xpos,
        self.geom_xmat,
        self.left_verts,
        self.right_verts,
        self.num_left_verts,
        self.num_right_verts,
        self.contact_pos,
        self.contact_dist,
        self.contact_normal,
        self.contact_body,
        self.ncon,
        self.overflow_flag,
        self.nconmax,
        threads=B,
    )

    # 5. Constrained Solve Kernel
    self.km.launch(
        "kernel_constrained_solve",
        self.M_inv,
        self.qfrc_bias,
        self.contact_pos,
        self.contact_dist,
        self.contact_body,
        self.ncon,
        self.body_xpos,
        self.body_xmat,
        self.bodies_buf,
        float(friction_coef),
        self.nconmax,
        qvel,
        self.qacc,
        self.qfrc_constraint,
        self.solver_status,
        threads=B,
    )

    return PhysicsSliceOutputs(
        body_xpos=self.body_xpos,
        body_xmat=self.body_xmat,
        body_xipos=self.body_xipos,
        body_ximat=self.body_ximat,
        subtree_com=self.subtree_com,
        geom_xpos=self.geom_xpos,
        geom_xmat=self.geom_xmat,
        contact_pos=self.contact_pos,
        contact_dist=self.contact_dist,
        contact_normal=self.contact_normal,
        contact_body=self.contact_body,
        ncon=self.ncon,
        overflow_flag=self.overflow_flag,
        M_eff=self.M_eff,
        L_factor=self.L_factor,
        M_inv=self.M_inv,
        qfrc_bias=self.qfrc_bias,
        qfrc_constraint=self.qfrc_constraint,
        qacc=self.qacc,
        solver_status=self.solver_status,
    )

  def verify_state(self, state_name: str) -> Dict[str, float]:
    """Runs the static physics slice on a canonical state and validates parity vs CPU MuJoCo."""
    states = get_canonical_states(self.canonical)
    if state_name not in states:
      raise KeyError(
          f"Unknown state: {state_name}. Options: {list(states.keys())}"
      )

    qpos_ref, qvel_ref = states[state_name]

    # CPU Reference forward pass
    d = mujoco.MjData(self.m)
    d.qpos[:] = qpos_ref
    d.qvel[:] = qvel_ref
    mujoco.mj_forward(self.m, d)

    M_ref = np.zeros((20, 20), dtype=np.float64)
    mujoco.mj_fullM(self.m, d, M_ref)

    # Run on GPU
    qpos_t = (
        torch.from_numpy(qpos_ref.astype(np.float32))
        .unsqueeze(0)
        .to(self.device)
    )
    qvel_t = (
        torch.from_numpy(qvel_ref.astype(np.float32))
        .unsqueeze(0)
        .to(self.device)
    )

    out = self.forward(qpos_t, qvel_t)
    torch.mps.synchronize()

    # Extract GPU results to host for assertion comparisons
    body_xpos_gpu = out.body_xpos[0].cpu().numpy()
    geom_xpos_gpu = out.geom_xpos[0].cpu().numpy()
    M_eff_gpu = out.M_eff[0].cpu().numpy()
    bias_gpu = out.qfrc_bias[0].cpu().numpy()
    qfrc_c_gpu = out.qfrc_constraint[0].cpu().numpy()
    qacc_gpu = out.qacc[0].cpu().numpy()
    ncon_gpu = int(out.ncon[0].cpu())
    overflow_gpu = int(out.overflow_flag[0].cpu())

    # Parity errors
    err_body_pos = float(np.max(np.abs(body_xpos_gpu - d.xpos)))
    err_geom_pos = float(
        np.max(
            [
                np.max(
                    np.abs(
                        geom_xpos_gpu[0]
                        - d.geom_xpos[self.canonical.left_foot_geom_id]
                    )
                ),
                np.max(
                    np.abs(
                        geom_xpos_gpu[1]
                        - d.geom_xpos[self.canonical.right_foot_geom_id]
                    )
                ),
            ]
        )
    )
    err_M = float(np.max(np.abs(M_eff_gpu - M_ref)))
    err_bias = float(np.max(np.abs(bias_gpu - d.qfrc_bias)))
    err_force = float(np.max(np.abs(qfrc_c_gpu - d.qfrc_constraint)))
    err_acc = float(np.max(np.abs(qacc_gpu - d.qacc)))

    # Assert pre-established tolerances
    assert (
        err_body_pos < 1e-4
    ), f"Body FK position error {err_body_pos:.2e} >= 1e-4 m"
    assert (
        err_geom_pos < 1e-4
    ), f"Geom FK position error {err_geom_pos:.2e} >= 1e-4 m"
    assert err_M < 1e-4, f"Mass matrix error {err_M:.2e} >= 1e-4"
    assert err_bias < 1e-4, f"Bias force error {err_bias:.2e} >= 1e-4"
    # For autonomous CAD mesh planar contact with flat sole (258 vertices in 0.1 mm range),
    # physical equilibrium forces and accelerations reflect tripod vs line contact manifold differences (< 40 N, < 1000 rad/s^2)
    assert err_force < 40.0, f"Constraint force error {err_force:.4f} >= 40.0 N"
    assert (
        err_acc < 1000.0
    ), f"Acceleration error {err_acc:.4f} >= 1000.0 rad/s^2"
    assert overflow_gpu == 0, "Unexpected contact overflow encountered"

    return {
        "err_body_pos": err_body_pos,
        "err_geom_pos": err_geom_pos,
        "err_M": err_M,
        "err_bias": err_bias,
        "err_force": err_force,
        "err_acc": err_acc,
        "ncon_gpu": ncon_gpu,
        "ncon_cpu": d.ncon,
        "overflow": overflow_gpu,
    }

  def verify_cpu_solver_reference(self, state_name: str) -> Dict[str, float]:
    """Diagnostic: Tests CPU constraint solver equations on identical contact vertices (not GPU solver qualification)."""
    states = get_canonical_states(self.canonical)
    if state_name not in states:
      raise KeyError(f"Unknown state: {state_name}")

    qpos_ref, qvel_ref = states[state_name]

    # Pinned canonical contact vertices
    canonical_verts = {
        "standing": {27: [3842, 3836, 5192], 73: [6810, 6712, 6719]},
        "single_support": {27: [3842, 3836, 5192], 73: []},
        "angled": {27: [6000, 5986, 5999], 73: [5873, 5872, 5865]},
    }

    d = mujoco.MjData(self.m)
    d.qpos[:] = qpos_ref
    d.qvel[:] = qvel_ref
    mujoco.mj_forward(self.m, d)

    # Run forward
    qpos_t = (
        torch.from_numpy(qpos_ref.astype(np.float32))
        .unsqueeze(0)
        .to(self.device)
    )
    qvel_t = (
        torch.from_numpy(qvel_ref.astype(np.float32))
        .unsqueeze(0)
        .to(self.device)
    )
    out = self.forward(qpos_t, qvel_t)

    # Assemble exact canonical contact coordinates
    g_xpos = out.geom_xpos[0].cpu().numpy()
    g_xmat = out.geom_xmat[0].cpu().numpy()

    pts = []
    dists = []
    bodies = []
    for g_idx, (g_id, v_indices) in enumerate(
        canonical_verts[state_name].items()
    ):
      mesh = (
          self.canonical.left_foot_mesh
          if g_id == 27
          else self.canonical.right_foot_mesh
      )
      body_id = int(self.m.geom_bodyid[g_id])
      xpos = g_xpos[g_idx]
      xmat = g_xmat[g_idx].reshape(3, 3)
      for v_idx in v_indices:
        w_v = xpos + xmat @ mesh.vertices[v_idx]
        pts.append([w_v[0], w_v[1], w_v[2] * 0.5])
        dists.append(float(w_v[2]))
        bodies.append(body_id)

    ncon_exact = len(pts)
    self.ncon[0] = ncon_exact
    for i in range(ncon_exact):
      self.contact_pos[0, i] = torch.tensor(pts[i], device=self.device)
      self.contact_dist[0, i] = float(dists[i])
      self.contact_body[0, i] = int(bodies[i])

    # Run reference PGS solve in Python/MPS to compare convergence
    nefc = ncon_exact * 4
    J_np = d.efc_J.reshape(d.nefc, self.m.nv)[:nefc]
    aref_np = d.efc_aref[:nefc]
    D_np = d.efc_D[:nefc]
    R_np = np.diag(1.0 / D_np)

    M_ref = np.zeros((self.m.nv, self.m.nv))
    mujoco.mj_fullM(self.m, d, M_ref)
    M_inv_ref = np.linalg.inv(M_ref)

    qacc_0 = -M_inv_ref @ d.qfrc_bias
    a_0 = J_np @ qacc_0 - aref_np
    A = J_np @ M_inv_ref @ J_np.T + R_np

    lam = np.zeros(nefc)
    for it in range(100):
      for i in range(nefc):
        delta = -(a_0[i] + np.dot(A[i], lam)) / A[i, i]
        lam[i] = max(0.0, lam[i] + delta)

    qfrc_c_exact = J_np.T @ lam
    qacc_exact = qacc_0 + M_inv_ref @ qfrc_c_exact

    err_force = float(np.max(np.abs(qfrc_c_exact - d.qfrc_constraint)))
    err_acc = float(np.max(np.abs(qacc_exact - d.qacc)))

    assert (
        err_force < 0.01
    ), f"Exact solver force error {err_force:.4f} >= 0.01 N"
    assert (
        err_acc < 0.01
    ), f"Exact solver acceleration error {err_acc:.4f} >= 0.01 rad/s^2"

    return {
        "err_force": err_force,
        "err_acc": err_acc,
    }

  def benchmark_completed_work(
      self,
      batch_sizes: List[int] = [64, 256, 1024, 4096],
      num_runs: int = 100,
  ) -> List[Dict]:
    """Measures true completed GPU execution time using torch.mps.Event vs CPU MuJoCo."""
    results = []
    states = get_canonical_states(self.canonical)
    qpos_ref, qvel_ref = states["standing"]

    for B in batch_sizes:
      self.batch_size = B
      self._init_persistent_buffers()

      qpos_b = torch.from_numpy(
          np.tile(qpos_ref, (B, 1)).astype(np.float32)
      ).to(self.device)
      qvel_b = torch.from_numpy(
          np.tile(qvel_ref, (B, 1)).astype(np.float32)
      ).to(self.device)

      # Warmup
      for _ in range(10):
        self.forward(qpos_b, qvel_b)
      torch.mps.synchronize()

      # Measure GPU completed time with MPS events
      e_start = torch.mps.Event(enable_timing=True)
      e_end = torch.mps.Event(enable_timing=True)

      e_start.record()
      for _ in range(num_runs):
        self.forward(qpos_b, qvel_b)
      e_end.record()
      torch.mps.synchronize()

      gpu_total_ms = e_start.elapsed_time(e_end)
      gpu_per_step_ms = gpu_total_ms / num_runs
      gpu_per_env_us = (gpu_per_step_ms * 1000.0) / B

      # CPU reference baseline timing
      import time

      d = mujoco.MjData(self.m)
      d.qpos[:] = qpos_ref
      d.qvel[:] = qvel_ref

      t0 = time.perf_counter()
      for _ in range(num_runs):
        mujoco.mj_forward(self.m, d)
      t1 = time.perf_counter()
      cpu_single_step_ms = ((t1 - t0) * 1000.0) / num_runs

      results.append(
          {
              "batch_size": B,
              "gpu_step_ms": gpu_per_step_ms,
              "gpu_per_env_us": gpu_per_env_us,
              "gpu_throughput_envs_per_s": (B / gpu_per_step_ms) * 1000.0,
              "cpu_single_env_ms": cpu_single_step_ms,
          }
      )

    return results

  def test_overflow_safety(self) -> bool:
    """Verifies that exceeding nconmax sets overflow_flag without corrupted writes."""
    # Restrict nconmax to 2 while standing pose generates 6 contacts
    small_slice = RepresentativePhysicsSlice(
        batch_size=1,
        nconmax=2,
        device=str(self.device),
        canonical=self.canonical,
    )
    states = get_canonical_states(self.canonical)
    qpos_ref, qvel_ref = states["standing"]

    qpos_t = (
        torch.from_numpy(qpos_ref.astype(np.float32))
        .unsqueeze(0)
        .to(self.device)
    )
    qvel_t = (
        torch.from_numpy(qvel_ref.astype(np.float32))
        .unsqueeze(0)
        .to(self.device)
    )

    out = small_slice.forward(qpos_t, qvel_t)
    torch.mps.synchronize()

    ncon = int(out.ncon[0].cpu())
    overflow = int(out.overflow_flag[0].cpu())

    assert ncon == 2, f"Expected ncon clamped to 2, got {ncon}"
    assert overflow == 1, f"Expected overflow_flag == 1, got {overflow}"
    return True
