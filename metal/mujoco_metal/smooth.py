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

"""CPU reference for rigid-body smooth mass matrix and inertial bias forces.

This stage ports the MuJoCo 3.10.0 comPos/comVel/CRBA/RNE structure. It
returns only dense generalized inertia and inertial/gravity bias. It does not
include actuator forces, passive forces, contacts, constraints, or integration.
"""

from dataclasses import fields

import mujoco
import numpy as np

from mujoco_metal.model import _validate_lowered
from mujoco_metal.model import ModelDescriptor


def _cross(a, b):
  return np.cross(a, b)


def _quat_matrix(q):
  w, x, y, z = q
  return np.array(
      [
          [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
          [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
          [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
      ]
  )


def _inertial_matrix(inertia, rotation, offset, mass):
  """Build the 6x6 spatial inertia matching MuJoCo's inertCom layout."""
  rot = rotation @ np.diag(inertia) @ rotation.T
  skew = np.array(
      [
          [0.0, -offset[2], offset[1]],
          [offset[2], 0.0, -offset[0]],
          [-offset[1], offset[0], 0.0],
      ]
  )
  upper_left = rot - mass * (skew @ skew)
  upper_right = mass * skew
  lower_left = -mass * skew
  lower_right = mass * np.eye(3)
  return np.block([[upper_left, upper_right], [lower_left, lower_right]])


def _motion_cross(a, b):
  return np.r_[
      _cross(a[:3], b[:3]), _cross(a[:3], b[3:]) + _cross(a[3:], b[:3])
  ]


def _force_cross(motion, force):
  return np.r_[
      _cross(motion[:3], force[:3]) + _cross(motion[3:], force[3:]),
      _cross(motion[:3], force[3:]),
  ]


def _dof_matrix(model, pose):
  """Create world-oriented DOFs about each body's free-root subtree COM."""
  centers = np.zeros((model.nbody, 3))
  for root in range(model.nbody):
    members = np.flatnonzero(model.body_rootid == root)
    total = float(np.sum(model.body_mass[members]))
    if total > 1e-15:
      centers[root] = (
          np.sum(
              pose["inertial_pos"][members] * model.body_mass[members, None],
              axis=0,
          )
          / total
      )
    elif members.size:
      centers[root] = pose["inertial_pos"][root]

  cdof = np.zeros((model.nv, 6), dtype=np.float64)
  for body in range(1, model.nbody):
    root = int(model.body_rootid[body])
    offset = centers[root] - pose["joint_anchor"]
    for joint in range(
        int(model.body_jntadr[body]),
        int(model.body_jntadr[body] + model.body_jntnum[body]),
    ):
      typ = int(model.jnt_type[joint])
      da = int(model.jnt_dofadr[joint])
      if typ == int(mujoco.mjtJoint.mjJNT_FREE):
        cdof[da : da + 3, 3:] = np.eye(3)
        rotation = _quat_matrix(pose["body_quat"][body])
        for axis in range(3):
          angular = rotation[:, axis]
          cdof[da + 3 + axis, :3] = angular
          cdof[da + 3 + axis, 3:] = _cross(angular, offset[joint])
      elif typ == int(mujoco.mjtJoint.mjJNT_BALL):
        rotation = _quat_matrix(pose["body_quat"][body])
        for axis in range(3):
          angular = rotation[:, axis]
          cdof[da + axis, :3] = angular
          cdof[da + axis, 3:] = _cross(angular, offset[joint])
      else:
        angular = pose["joint_axis"][joint]
        if typ == int(mujoco.mjtJoint.mjJNT_HINGE):
          cdof[da, :3] = angular
          cdof[da, 3:] = _cross(angular, offset[joint])
        else:
          cdof[da, 3:] = angular
  return centers, cdof


def _spatial_inertias(model, pose, centers):
  inertia = np.zeros((model.nbody, 6, 6), dtype=np.float64)
  for body in range(1, model.nbody):
    root = int(model.body_rootid[body])
    offset = pose["inertial_pos"][body] - centers[root]
    rotation = _quat_matrix(pose["inertial_quat"][body])
    inertia[body] = _inertial_matrix(
        model.body_inertia[body], rotation, offset, model.body_mass[body]
    )
  return inertia


def smooth_dynamics(model: ModelDescriptor, qpos, qvel):
  """Return dense ``M(q)`` and inertial/gravity ``qfrc_bias`` on the CPU.

  Only models without actuators are accepted because MuJoCo actuator-armature
  contributions are outside this stage. Joint armature and gravity are
  included. The implementation is a CPU reference, not a Metal execution path.
  """
  if model.nu:
    raise ValueError("smooth dynamics reference does not support actuators")
  counts = {
      name: getattr(model, name)
      for name in ("nq", "nv", "nbody", "njnt", "ngeom", "nsite")
  }
  values = {
      item.name: getattr(model, item.name)
      for item in fields(ModelDescriptor)
      if isinstance(getattr(model, item.name), np.ndarray)
  }
  _validate_lowered(counts, values)
  qpos = np.asarray(qpos, dtype=np.float64)
  qvel = np.asarray(qvel, dtype=np.float64)
  if qpos.shape != (model.nq,) or not np.all(np.isfinite(qpos)):
    raise ValueError(f"qpos must be finite with shape ({model.nq},)")
  if qvel.shape != (model.nv,) or not np.all(np.isfinite(qvel)):
    raise ValueError(f"qvel must be finite with shape ({model.nv},)")

  pose = model.forward_kinematics(qpos)
  centers, cdof = _dof_matrix(model, pose)
  local_inertia = _spatial_inertias(model, pose, centers)
  composite = local_inertia.copy()
  for body in range(model.nbody - 1, 0, -1):
    parent = int(model.body_parentid[body])
    if parent:
      composite[parent] += composite[body]

  mass_matrix = np.zeros((model.nv, model.nv), dtype=np.float64)
  for dof in range(model.nv):
    body = int(model.dof_bodyid[dof])
    projected = composite[body] @ cdof[dof]
    ancestor = dof
    while ancestor >= 0:
      value = float(cdof[ancestor] @ projected)
      mass_matrix[dof, ancestor] = value
      mass_matrix[ancestor, dof] = value
      ancestor = int(model.dof_parentid[ancestor])
    mass_matrix[dof, dof] += model.dof_armature[dof]

  cvel = np.zeros((model.nbody, 6), dtype=np.float64)
  cdof_dot = np.zeros((model.nv, 6), dtype=np.float64)
  for body in range(1, model.nbody):
    velocity = cvel[int(model.body_parentid[body])].copy()
    count = int(model.body_dofnum[body])
    cursor = 0
    joints = range(
        int(model.body_jntadr[body]),
        int(model.body_jntadr[body] + model.body_jntnum[body]),
    )
    for joint in joints:
      typ = int(model.jnt_type[joint])
      start = int(model.jnt_dofadr[joint])
      width = (
          6
          if typ == int(mujoco.mjtJoint.mjJNT_FREE)
          else (3 if typ == int(mujoco.mjtJoint.mjJNT_BALL) else 1)
      )
      if typ in (
          int(mujoco.mjtJoint.mjJNT_FREE),
          int(mujoco.mjtJoint.mjJNT_BALL),
      ):
        translational = 3 if typ == int(mujoco.mjtJoint.mjJNT_FREE) else 0
        velocity += (
            cdof[start : start + translational].T
            @ qvel[start : start + translational]
        )
        angular_start = start + translational
        for offset in range(translational, width):
          dof = start + offset
          cdof_dot[dof] = _motion_cross(velocity, cdof[dof])
        velocity += (
            cdof[angular_start : start + width].T
            @ qvel[angular_start : start + width]
        )
      else:
        dof = start
        cdof_dot[dof] = _motion_cross(velocity, cdof[dof])
        velocity += cdof[dof] * qvel[dof]
      cursor += width
    if cursor != count:
      raise ValueError(f"inconsistent body DOF range for body {body}")
    cvel[body] = velocity

  cacc = np.zeros((model.nbody, 6), dtype=np.float64)
  if not (model.disableflags & int(mujoco.mjtDisableBit.mjDSBL_GRAVITY)):
    cacc[0, 3:] = -model.gravity
  body_force = np.zeros((model.nbody, 6), dtype=np.float64)
  for body in range(1, model.nbody):
    parent = int(model.body_parentid[body])
    start = int(model.body_dofadr[body])
    count = int(model.body_dofnum[body])
    cacc[body] = (
        cacc[parent]
        + cdof_dot[start : start + count].T @ qvel[start : start + count]
    )
    momentum = local_inertia[body] @ cvel[body]
    body_force[body] = local_inertia[body] @ cacc[body] + _force_cross(
        cvel[body], momentum
    )
  for body in range(model.nbody - 1, 0, -1):
    parent = int(model.body_parentid[body])
    if parent:
      body_force[parent] += body_force[body]
  bias = np.zeros(model.nv, dtype=np.float64)
  for dof in range(model.nv):
    bias[dof] = cdof[dof] @ body_force[int(model.dof_bodyid[dof])]
  return {"mass_matrix": mass_matrix, "qfrc_bias": bias}
