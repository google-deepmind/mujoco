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

"""Canonical Model Loader for MicroDuck Physics Slice.

Dynamically resolves bodies, joints, geoms, meshes, and simulation parameters
by name from the compiled canonical model. Provides reproducible reference states
and verifies that only asserted contact pairs (terrain vs foot geoms) are active.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import mujoco
import numpy as np

CANONICAL_XML_PATH = (
    Path(__file__).resolve().parent / "assets" / "microduck_canonical_flat.xml"
)


@dataclass
class FootMeshData:
  geom_id: int
  geom_name: str
  body_id: int
  dataid: int
  vertadr: int
  vertnum: int
  graphadr: int
  vertices: np.ndarray  # (vertnum, 3) in local geom frame
  graph: np.ndarray  # 1D int32 array of graph slice for this mesh
  rbound: float  # geom bounding radius


@dataclass
class CanonicalMicroDuckModel:
  model: mujoco.MjModel
  nq: int
  nv: int
  nu: int
  nbody: int
  njnt: int
  ngeom: int
  # Dynamically resolved IDs
  base_body_id: int
  terrain_geom_id: int
  left_foot_geom_id: int
  right_foot_geom_id: int
  left_foot_body_id: int
  right_foot_body_id: int
  # Foot CAD meshes
  left_foot_mesh: FootMeshData
  right_foot_mesh: FootMeshData
  # Mechanical parameters
  dof_armature: np.ndarray  # (20,)
  body_mass: np.ndarray  # (nbody,)
  body_inertia: np.ndarray  # (nbody, 3)
  body_pos: np.ndarray  # (nbody, 3)
  body_quat: np.ndarray  # (nbody, 4)
  body_parent: np.ndarray  # (nbody,)
  jnt_type: np.ndarray  # (njnt,)
  jnt_qposadr: np.ndarray  # (njnt,)
  jnt_dofadr: np.ndarray  # (njnt,)
  jnt_axis: np.ndarray  # (njnt, 3)
  jnt_pos: np.ndarray  # (njnt, 3)
  jnt_bodyid: np.ndarray  # (njnt,)


def _extract_mesh_graph_slice(m: mujoco.MjModel, dataid: int) -> np.ndarray:
  gadr = m.mesh_graphadr[dataid]
  if gadr < 0:
    return np.zeros(0, dtype=np.int32)
  numvert = m.mesh_graph[gadr]
  vert_edgeadr = m.mesh_graph[gadr + 2 : gadr + 2 + numvert]
  last_adr = vert_edgeadr[-1]
  edge_start = gadr + 2 + 2 * numvert
  cur = edge_start + last_adr
  while m.mesh_graph[cur] >= 0:
    cur += 1
  total_ints = cur + 1 - gadr
  return m.mesh_graph[gadr : gadr + total_ints].copy().astype(np.int32)


def load_canonical_model() -> CanonicalMicroDuckModel:
  if not CANONICAL_XML_PATH.exists():
    raise FileNotFoundError(f"Canonical XML not found at {CANONICAL_XML_PATH}")

  m = mujoco.MjModel.from_xml_path(str(CANONICAL_XML_PATH))

  # Resolve body IDs dynamically
  base_body_id = mujoco.mj_name2id(
      m, mujoco.mjtObj.mjOBJ_BODY, "robot/trunk_base"
  )
  left_foot_body_id = mujoco.mj_name2id(
      m, mujoco.mjtObj.mjOBJ_BODY, "robot/ankle_left"
  )
  right_foot_body_id = mujoco.mj_name2id(
      m, mujoco.mjtObj.mjOBJ_BODY, "robot/ankle_right"
  )

  assert base_body_id >= 0, "robot/trunk_base body not found"
  assert left_foot_body_id >= 0, "robot/ankle_left body not found"
  assert right_foot_body_id >= 0, "robot/ankle_right body not found"

  # Resolve geom IDs dynamically
  terrain_geom_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "terrain")
  left_foot_geom_id = mujoco.mj_name2id(
      m, mujoco.mjtObj.mjOBJ_GEOM, "robot/left_foot_collision"
  )
  right_foot_geom_id = mujoco.mj_name2id(
      m, mujoco.mjtObj.mjOBJ_GEOM, "robot/right_foot_collision"
  )

  assert terrain_geom_id >= 0, "terrain geom not found"
  assert left_foot_geom_id >= 0, "robot/left_foot_collision geom not found"
  assert right_foot_geom_id >= 0, "robot/right_foot_collision geom not found"

  # Extract left foot CAD mesh
  l_dataid = m.geom_dataid[left_foot_geom_id]
  l_vadr = m.mesh_vertadr[l_dataid]
  l_vnum = m.mesh_vertnum[l_dataid]
  l_gadr = m.mesh_graphadr[l_dataid]
  l_verts = m.mesh_vert[l_vadr : l_vadr + l_vnum].copy()
  left_mesh = FootMeshData(
      geom_id=left_foot_geom_id,
      geom_name="robot/left_foot_collision",
      body_id=left_foot_body_id,
      dataid=l_dataid,
      vertadr=l_vadr,
      vertnum=l_vnum,
      graphadr=l_gadr,
      vertices=l_verts,
      graph=_extract_mesh_graph_slice(m, l_dataid),
      rbound=float(m.geom_rbound[left_foot_geom_id]),
  )

  # Extract right foot CAD mesh
  r_dataid = m.geom_dataid[right_foot_geom_id]
  r_vadr = m.mesh_vertadr[r_dataid]
  r_vnum = m.mesh_vertnum[r_dataid]
  r_gadr = m.mesh_graphadr[r_dataid]
  r_verts = m.mesh_vert[r_vadr : r_vadr + r_vnum].copy()
  right_mesh = FootMeshData(
      geom_id=right_foot_geom_id,
      geom_name="robot/right_foot_collision",
      body_id=right_foot_body_id,
      dataid=r_dataid,
      vertadr=r_vadr,
      vertnum=r_vnum,
      graphadr=r_gadr,
      vertices=r_verts,
      graph=_extract_mesh_graph_slice(m, r_dataid),
      rbound=float(m.geom_rbound[right_foot_geom_id]),
  )

  return CanonicalMicroDuckModel(
      model=m,
      nq=m.nq,
      nv=m.nv,
      nu=m.nu,
      nbody=m.nbody,
      njnt=m.njnt,
      ngeom=m.ngeom,
      base_body_id=base_body_id,
      terrain_geom_id=terrain_geom_id,
      left_foot_geom_id=left_foot_geom_id,
      right_foot_geom_id=right_foot_geom_id,
      left_foot_body_id=left_foot_body_id,
      right_foot_body_id=right_foot_body_id,
      left_foot_mesh=left_mesh,
      right_foot_mesh=right_mesh,
      dof_armature=m.dof_armature.copy(),
      body_mass=m.body_mass.copy(),
      body_inertia=m.body_inertia.copy(),
      body_pos=m.body_pos.copy(),
      body_quat=m.body_quat.copy(),
      body_parent=m.body_parentid.copy(),
      jnt_type=m.jnt_type.copy(),
      jnt_qposadr=m.jnt_qposadr.copy(),
      jnt_dofadr=m.jnt_dofadr.copy(),
      jnt_axis=m.jnt_axis.copy(),
      jnt_pos=m.jnt_pos.copy(),
      jnt_bodyid=m.jnt_bodyid.copy(),
  )


def get_canonical_states(
    canonical: CanonicalMicroDuckModel,
) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
  """Generates the three asserted reproducible static states.

  1. 'standing': Dual support, keyframe 0, zero velocity.
  2. 'single_support': Lifted right leg in the air, active left foot contact only.
  3. 'angled': 15-degree rolled base, testing tilted CAD foot contacts.
  """
  m = canonical.model
  states = {}

  # State 1: Standing
  qpos_standing = m.key_qpos[0].copy()
  qvel_standing = np.zeros(m.nv, dtype=np.float64)
  states["standing"] = (qpos_standing, qvel_standing)

  # State 2: Single Support (lift right hip pitch and knee)
  qpos_single = m.key_qpos[0].copy()
  j_r_pitch = mujoco.mj_name2id(
      m, mujoco.mjtObj.mjOBJ_JOINT, "robot/right_hip_pitch"
  )
  j_r_knee = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "robot/right_knee")
  qadr_pitch = m.jnt_qposadr[j_r_pitch]
  qadr_knee = m.jnt_qposadr[j_r_knee]
  qpos_single[qadr_pitch] = -1.2
  qpos_single[qadr_knee] = 1.5
  qvel_single = np.full(m.nv, 0.05, dtype=np.float64)
  states["single_support"] = (qpos_single, qvel_single)

  # State 3: Angled Base (15 degree roll)
  qpos_angled = m.key_qpos[0].copy()
  angle = np.radians(15.0)
  qpos_angled[3] = np.cos(angle / 2.0)
  qpos_angled[4] = np.sin(angle / 2.0)
  qpos_angled[5] = 0.0
  qpos_angled[6] = 0.0
  qvel_angled = np.zeros(m.nv, dtype=np.float64)
  states["angled"] = (qpos_angled, qvel_angled)

  return states


def verify_asserted_contacts(canonical: CanonicalMicroDuckModel):
  """Asserts that all contacts in the three test states are ONLY terrain vs feet."""
  m = canonical.model
  d = mujoco.MjData(m)
  states = get_canonical_states(canonical)

  valid_pairs = {
      (canonical.terrain_geom_id, canonical.left_foot_geom_id),
      (canonical.left_foot_geom_id, canonical.terrain_geom_id),
      (canonical.terrain_geom_id, canonical.right_foot_geom_id),
      (canonical.right_foot_geom_id, canonical.terrain_geom_id),
  }

  for name, (qpos, qvel) in states.items():
    d.qpos[:] = qpos
    d.qvel[:] = qvel
    mujoco.mj_forward(m, d)

    assert (
        d.ncon > 0
    ), f"State '{name}' produced 0 contacts; expected active ground contacts."
    pairs = set((d.contact[i].geom1, d.contact[i].geom2) for i in range(d.ncon))
    unsupported = pairs - valid_pairs
    assert not unsupported, (
        f"State '{name}' produced unexpected contact pairs {unsupported}. "
        f"Allowed pairs: {valid_pairs}"
    )
    if name == "single_support":
      right_contacts = [
          i
          for i in range(d.ncon)
          if canonical.right_foot_geom_id
          in (d.contact[i].geom1, d.contact[i].geom2)
      ]
      assert (
          len(right_contacts) == 0
      ), f"State 'single_support' expected 0 right foot contacts, got {len(right_contacts)}"


def create_matching_cpu_model(
    canonical: CanonicalMicroDuckModel,
    d_npz: Dict[str, np.ndarray],
    xml_path: Optional[Path] = None,
) -> mujoco.MjModel:
  """Constructs a fresh CPU MjModel exactly matching scenario-randomized parameters.

  Copies per-world body masses, CoM offsets (ipos), armature, and geom friction
  so that CPU reference evaluation mirrors GPU inputs down to numerical precision.
  """
  path = xml_path or CANONICAL_XML_PATH
  m = mujoco.MjModel.from_xml_path(str(path))
  if "per_world_mass" in d_npz and np.any(d_npz["per_world_mass"] != 0):
    m.body_mass[:] = d_npz["per_world_mass"]
  if "per_world_ipos" in d_npz and np.any(d_npz["per_world_ipos"] != 0):
    m.body_ipos[:] = d_npz["per_world_ipos"]
  if "per_world_armature" in d_npz and np.any(d_npz["per_world_armature"] != 0):
    m.dof_armature[:] = d_npz["per_world_armature"]
  if "contact_friction" in d_npz and len(d_npz["contact_friction"]) > 0:
    fric = float(d_npz["contact_friction"][0, 0])
    m.geom_friction[canonical.terrain_geom_id, 0] = fric
    m.geom_friction[canonical.left_foot_geom_id, 0] = fric
    m.geom_friction[canonical.right_foot_geom_id, 0] = fric
  return m


def create_matching_cpu_data(
    m: mujoco.MjModel,
    d_npz: Dict[str, np.ndarray],
) -> mujoco.MjData:
  """Creates a fresh MjData instance initialized with scenario qpos, qvel, and qfrc_applied."""
  d = mujoco.MjData(m)
  d.qpos[:] = d_npz["qpos"]
  d.qvel[:] = d_npz["qvel"]
  if "qfrc_applied" in d_npz and np.any(d_npz["qfrc_applied"] != 0):
    d.qfrc_applied[:] = d_npz["qfrc_applied"]
  return d
