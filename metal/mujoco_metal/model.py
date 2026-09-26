"""Immutable, dimension-derived model lowering and CPU forward kinematics."""

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np


def _frozen(array, dtype=None):
  value = np.asarray(array, dtype=dtype, order="C")
  return np.frombuffer(value.tobytes(), dtype=value.dtype).reshape(value.shape)


def _quat_mul(a, b):
  aw, ax, ay, az = a
  bw, bx, by, bz = b
  return np.array([aw*bw-ax*bx-ay*by-az*bz,
                   aw*bx+ax*bw+ay*bz-az*by,
                   aw*by-ax*bz+ay*bw+az*bx,
                   aw*bz+ax*by-ay*bx+az*bw])


def _rotate(q, v):
  p = np.array([0., *v])
  return _quat_mul(_quat_mul(q, p), q * np.array([1., -1., -1., -1.]))[1:]


def _axis_quat(axis, angle):
  axis = np.asarray(axis, dtype=np.float64)
  norm = np.linalg.norm(axis)
  if norm == 0:
    raise ValueError("joint axis must be nonzero")
  return np.r_[np.cos(angle/2), axis/norm*np.sin(angle/2)]


def _unit(q, what):
  q = np.asarray(q, dtype=np.float64)
  norm = np.linalg.norm(q)
  if not np.isfinite(norm) or norm <= 1e-15:
    raise ValueError(f"{what} quaternion must be finite and nonzero")
  return q / norm


@dataclass(frozen=True)
class ModelDescriptor:
  """Owned compiled-model constants, safe to share between simulation states."""

  nq: int
  nv: int
  nbody: int
  njnt: int
  ngeom: int
  nsite: int
  body_parentid: np.ndarray
  body_pos: np.ndarray
  body_quat: np.ndarray
  body_ipos: np.ndarray
  body_iquat: np.ndarray
  jnt_type: np.ndarray
  jnt_qposadr: np.ndarray
  jnt_dofadr: np.ndarray
  jnt_bodyid: np.ndarray
  jnt_pos: np.ndarray
  jnt_axis: np.ndarray
  qpos0: np.ndarray
  geom_bodyid: np.ndarray
  geom_pos: np.ndarray
  geom_quat: np.ndarray
  site_bodyid: np.ndarray
  site_pos: np.ndarray
  site_quat: np.ndarray

  def forward_kinematics(self, qpos):
    """Return body, geom, site and inertial world poses for one qpos vector."""
    qpos = np.asarray(qpos, dtype=np.float64)
    if qpos.shape != (self.nq,) or not np.all(np.isfinite(qpos)):
      raise ValueError(f"qpos must be finite with shape ({self.nq},)")
    bp = np.zeros((self.nbody, 3)); bq = np.zeros((self.nbody, 4))
    bq[:, 0] = 1.
    joints_by_body = [[] for _ in range(self.nbody)]
    for j, body in enumerate(self.jnt_bodyid):
      joints_by_body[int(body)].append(j)
    for body in range(1, self.nbody):
      parent = int(self.body_parentid[body])
      bp[body] = bp[parent] + _rotate(bq[parent], self.body_pos[body])
      bq[body] = _quat_mul(bq[parent], self.body_quat[body])
      for j in joints_by_body[body]:
        typ = int(self.jnt_type[j]); qa = int(self.jnt_qposadr[j])
        if typ == int(mujoco.mjtJoint.mjJNT_FREE):
          bp[body] = qpos[qa:qa+3]
          bq[body] = _unit(qpos[qa+3:qa+7], "free joint")
        elif typ == int(mujoco.mjtJoint.mjJNT_BALL):
          anchor = bp[body] + _rotate(bq[body], self.jnt_pos[j])
          rotation = _unit(qpos[qa:qa+4], "ball joint")
          bq[body] = _quat_mul(bq[body], rotation)
          bp[body] = anchor - _rotate(bq[body], self.jnt_pos[j])
        elif typ in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)):
          value = qpos[qa] - self.qpos0[qa]
          if typ == int(mujoco.mjtJoint.mjJNT_SLIDE):
            bp[body] += _rotate(bq[body], self.jnt_axis[j] * value)
          else:
            anchor = bp[body] + _rotate(bq[body], self.jnt_pos[j])
            bq[body] = _quat_mul(bq[body], _axis_quat(self.jnt_axis[j], value))
            bp[body] = anchor - _rotate(bq[body], self.jnt_pos[j])
    gp, gq = _attached_poses(bp, bq, self.geom_bodyid, self.geom_pos, self.geom_quat)
    sp, sq = _attached_poses(bp, bq, self.site_bodyid, self.site_pos, self.site_quat)
    ip, iq = _attached_poses(bp, bq, np.arange(self.nbody), self.body_ipos, self.body_iquat)
    return {"body_pos": bp, "body_quat": bq, "geom_pos": gp, "geom_quat": gq,
            "site_pos": sp, "site_quat": sq, "inertial_pos": ip, "inertial_quat": iq}


def _attached_poses(bp, bq, bodyids, pos, quat):
  outp = np.empty((len(bodyids), 3)); outq = np.empty((len(bodyids), 4))
  for i, body in enumerate(bodyids):
    outp[i] = bp[body] + _rotate(bq[body], pos[i])
    outq[i] = _quat_mul(bq[body], quat[i])
  return outp, outq


def _validate_lowered(counts, values):
  """Check compiled addressing before arrays can be consumed by a kernel."""
  nb, nj, nq, nv = counts["nbody"], counts["njnt"], counts["nq"], counts["nv"]
  parent = values["body_parentid"]
  if parent.shape != (nb,) or parent[0] != 0:
    raise ValueError("invalid body parent array")
  if np.any(parent[1:] < 0) or np.any(parent[1:] >= np.arange(1, nb)):
    raise ValueError("body parents must precede their children")
  for i, (typ, qa, da, body) in enumerate(zip(values["jnt_type"], values["jnt_qposadr"], values["jnt_dofadr"], values["jnt_bodyid"])):
    qwidth, dwidth = {0: (7, 6), 1: (4, 3), 2: (1, 1), 3: (1, 1)}.get(int(typ), (0, 0))
    if not qwidth or qa < 0 or qa + qwidth > nq or da < 0 or da + dwidth > nv:
      raise ValueError(f"invalid joint type or address at joint {i}")
    if body <= 0 or body >= nb:
      raise ValueError(f"invalid joint body at joint {i}")
  if values["qpos0"].shape != (nq,):
    raise ValueError("invalid qpos0 shape")


def load_model(source):
  """Compile XML/path/bytes with pinned MuJoCo and lower all FK constants."""
  if isinstance(source, mujoco.MjModel):
    m = source
  elif isinstance(source, bytes):
    m = mujoco.MjModel.from_xml_string(source.decode())
  elif isinstance(source, Path) or (isinstance(source, str) and "<" not in source):
    m = mujoco.MjModel.from_xml_path(str(source))
  else:
    m = mujoco.MjModel.from_xml_string(str(source))
  if mujoco.__version__ != "3.10.0":
    raise RuntimeError(f"requires MuJoCo 3.10.0; found {mujoco.__version__}")
  values = {}
  for name in ("body_parentid", "body_pos", "body_quat", "body_ipos", "body_iquat",
               "jnt_type", "jnt_qposadr", "jnt_dofadr", "jnt_bodyid", "jnt_pos",
               "jnt_axis", "qpos0", "geom_bodyid", "geom_pos", "geom_quat",
               "site_bodyid", "site_pos", "site_quat"):
    values[name] = _frozen(getattr(m, name))
  counts = dict(nq=m.nq, nv=m.nv, nbody=m.nbody, njnt=m.njnt, ngeom=m.ngeom, nsite=m.nsite)
  _validate_lowered(counts, values)
  return ModelDescriptor(**counts, **values)
