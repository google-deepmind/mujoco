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
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def _rotate(q, v):
    p = np.array([0.0, *v])
    return _quat_mul(_quat_mul(q, p), q * np.array([1.0, -1.0, -1.0, -1.0]))[1:]


def _axis_quat(axis, angle):
    axis = np.asarray(axis, dtype=np.float64)
    norm = np.linalg.norm(axis)
    if norm == 0:
        raise ValueError("joint axis must be nonzero")
    return np.r_[np.cos(angle / 2), axis / norm * np.sin(angle / 2)]


def _unit(q, what):
    q = np.asarray(q, dtype=np.float64)
    if not np.all(np.isfinite(q)):
        raise ValueError(f"{what} quaternion must be finite and nonzero")
    scale = np.max(np.abs(q))
    if scale == 0:
        raise ValueError(f"{what} quaternion must be finite and nonzero")
    scaled = q / scale
    return scaled / np.linalg.norm(scaled)


@dataclass(frozen=True)
class ModelDescriptor:
    """Owned compiled-model constants, safe to share between simulation states."""

    nq: int
    nv: int
    nmocap: int
    nbody: int
    njnt: int
    ngeom: int
    nsite: int
    body_parentid: np.ndarray
    body_jntadr: np.ndarray
    body_jntnum: np.ndarray
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
        bp = np.zeros((self.nbody, 3))
        bq = np.zeros((self.nbody, 4))
        bq[:, 0] = 1.0
        joints_by_body = [[] for _ in range(self.nbody)]
        for j, body in enumerate(self.jnt_bodyid):
            joints_by_body[int(body)].append(j)
        for body in range(1, self.nbody):
            parent = int(self.body_parentid[body])
            bp[body] = bp[parent] + _rotate(bq[parent], self.body_pos[body])
            bq[body] = _unit(_quat_mul(bq[parent], self.body_quat[body]), "body")
            for j in joints_by_body[body]:
                typ = int(self.jnt_type[j])
                qa = int(self.jnt_qposadr[j])
                if typ == int(mujoco.mjtJoint.mjJNT_FREE):
                    bp[body] = qpos[qa : qa + 3]
                    bq[body] = _unit(qpos[qa + 3 : qa + 7], "free joint")
                elif typ == int(mujoco.mjtJoint.mjJNT_BALL):
                    anchor = bp[body] + _rotate(bq[body], self.jnt_pos[j])
                    rotation = _unit(qpos[qa : qa + 4], "ball joint")
                    bq[body] = _unit(_quat_mul(bq[body], rotation), "body")
                    bp[body] = anchor - _rotate(bq[body], self.jnt_pos[j])
                elif typ in (
                    int(mujoco.mjtJoint.mjJNT_HINGE),
                    int(mujoco.mjtJoint.mjJNT_SLIDE),
                ):
                    value = qpos[qa] - self.qpos0[qa]
                    if typ == int(mujoco.mjtJoint.mjJNT_SLIDE):
                        bp[body] += _rotate(bq[body], self.jnt_axis[j] * value)
                    else:
                        anchor = bp[body] + _rotate(bq[body], self.jnt_pos[j])
                        bq[body] = _unit(
                            _quat_mul(bq[body], _axis_quat(self.jnt_axis[j], value)),
                            "body",
                        )
                        bp[body] = anchor - _rotate(bq[body], self.jnt_pos[j])
        gp, gq = _attached_poses(
            bp, bq, self.geom_bodyid, self.geom_pos, self.geom_quat
        )
        sp, sq = _attached_poses(
            bp, bq, self.site_bodyid, self.site_pos, self.site_quat
        )
        ip, iq = _attached_poses(
            bp, bq, np.arange(self.nbody), self.body_ipos, self.body_iquat
        )
        return {
            "body_pos": bp,
            "body_quat": bq,
            "geom_pos": gp,
            "geom_quat": gq,
            "site_pos": sp,
            "site_quat": sq,
            "inertial_pos": ip,
            "inertial_quat": iq,
        }


def _attached_poses(bp, bq, bodyids, pos, quat):
    outp = np.empty((len(bodyids), 3))
    outq = np.empty((len(bodyids), 4))
    for i, body in enumerate(bodyids):
        outp[i] = bp[body] + _rotate(bq[body], pos[i])
        outq[i] = _quat_mul(bq[body], quat[i])
    return outp, outq


def _validate_lowered(counts, values):
    """Check compiled addressing before arrays can be consumed by a kernel."""
    nb, nj, nq, nv = counts["nbody"], counts["njnt"], counts["nq"], counts["nv"]
    parent = values["body_parentid"]
    array_shapes = {
        "body_jntadr": (nb,),
        "body_jntnum": (nb,),
        "body_pos": (nb, 3),
        "body_quat": (nb, 4),
        "body_ipos": (nb, 3),
        "body_iquat": (nb, 4),
        "jnt_type": (nj,),
        "jnt_qposadr": (nj,),
        "jnt_dofadr": (nj,),
        "jnt_bodyid": (nj,),
        "jnt_pos": (nj, 3),
        "jnt_axis": (nj, 3),
        "geom_bodyid": (counts["ngeom"],),
        "geom_pos": (counts["ngeom"], 3),
        "geom_quat": (counts["ngeom"], 4),
        "site_bodyid": (counts["nsite"],),
        "site_pos": (counts["nsite"], 3),
        "site_quat": (counts["nsite"], 4),
    }
    for name, shape in array_shapes.items():
        if values[name].shape != shape:
            raise ValueError(
                f"invalid {name} shape: {values[name].shape}, expected {shape}"
            )
        if values[name].dtype.kind in "fc" and not np.all(np.isfinite(values[name])):
            raise ValueError(f"nonfinite values in {name}")
    for name in ("body_quat", "body_iquat", "geom_quat", "site_quat"):
        norms = np.linalg.norm(values[name], axis=1)
        if np.any(np.abs(norms - 1.0) > 1e-6):
            raise ValueError(f"{name} contains a non-unit quaternion")
    for name in ("geom_bodyid", "site_bodyid"):
        if np.any(values[name] < 0) or np.any(values[name] >= nb):
            raise ValueError(f"invalid body index in {name}")
    if parent.shape != (nb,) or parent[0] != 0:
        raise ValueError("invalid body parent array")
    if np.any(parent[1:] < 0) or np.any(parent[1:] >= np.arange(1, nb)):
        raise ValueError("body parents must precede their children")
    qcovered = np.zeros(nq, dtype=np.int8)
    dcovered = np.zeros(nv, dtype=np.int8)
    free_bodies = set()
    for i, (typ, qa, da, body) in enumerate(
        zip(
            values["jnt_type"],
            values["jnt_qposadr"],
            values["jnt_dofadr"],
            values["jnt_bodyid"],
        )
    ):
        qwidth, dwidth = {0: (7, 6), 1: (4, 3), 2: (1, 1), 3: (1, 1)}.get(
            int(typ), (0, 0)
        )
        if not qwidth or qa < 0 or qa + qwidth > nq or da < 0 or da + dwidth > nv:
            raise ValueError(f"invalid joint type or address at joint {i}")
        if body <= 0 or body >= nb:
            raise ValueError(f"invalid joint body at joint {i}")
        if qcovered[qa : qa + qwidth].any() or dcovered[da : da + dwidth].any():
            raise ValueError(f"overlapping joint address at joint {i}")
        qcovered[qa : qa + qwidth] = 1
        dcovered[da : da + dwidth] = 1
        if typ == 0:
            if int(body) in free_bodies or np.any(values["jnt_bodyid"][:i] == body):
                raise ValueError("free joint cannot share its body with another joint")
            free_bodies.add(int(body))
        elif int(body) in free_bodies:
            raise ValueError("free joint cannot share its body with another joint")
        if typ in (2, 3):
            axis_norm = np.linalg.norm(values["jnt_axis"][i])
            if abs(axis_norm - 1.0) > 1e-6:
                raise ValueError(f"joint axis must be unit length at joint {i}")
        if typ == 0 and parent[body] != 0:
            raise ValueError("free joint body must be a direct child of world")
    if not np.all(qcovered) or not np.all(dcovered):
        raise ValueError("joint addresses must exhaustively cover qpos and dof arrays")
    range_coverage = np.zeros(nj, dtype=np.int8)
    for body, (start, count) in enumerate(
        zip(values["body_jntadr"], values["body_jntnum"])
    ):
        if (
            count < 0
            or (count == 0 and (start < -1 or start > nj))
            or (count > 0 and (start < 0 or start + count > nj))
        ):
            raise ValueError(f"invalid joint range for body {body}")
        if count:
            if np.any(values["jnt_bodyid"][start : start + count] != body):
                raise ValueError(
                    f"joint range does not match body association for body {body}"
                )
            if np.any(range_coverage[start : start + count]):
                raise ValueError(f"overlapping joint ranges at body {body}")
            range_coverage[start : start + count] = 1
    if not np.all(range_coverage):
        raise ValueError("body joint ranges do not cover the joint array")
    if values["qpos0"].shape != (nq,) or not np.all(np.isfinite(values["qpos0"])):
        raise ValueError("invalid or nonfinite qpos0")


def load_model(source):
    """Compile XML/path/bytes with pinned MuJoCo and lower all FK constants."""
    if mujoco.__version__ != "3.10.0":
        raise RuntimeError(f"requires MuJoCo 3.10.0; found {mujoco.__version__}")
    if isinstance(source, mujoco.MjModel):
        m = source
    elif isinstance(source, bytes):
        m = mujoco.MjModel.from_xml_string(source.decode())
    elif isinstance(source, Path) or (isinstance(source, str) and "<" not in source):
        m = mujoco.MjModel.from_xml_path(str(source))
    else:
        m = mujoco.MjModel.from_xml_string(str(source))
    if m.nmocap:
        raise ValueError("mocap inputs are unsupported by the current kinematics stage")
    values = {}
    for name in (
        "body_parentid",
        "body_jntadr",
        "body_jntnum",
        "body_pos",
        "body_quat",
        "body_ipos",
        "body_iquat",
        "jnt_type",
        "jnt_qposadr",
        "jnt_dofadr",
        "jnt_bodyid",
        "jnt_pos",
        "jnt_axis",
        "qpos0",
        "geom_bodyid",
        "geom_pos",
        "geom_quat",
        "site_bodyid",
        "site_pos",
        "site_quat",
    ):
        values[name] = _frozen(getattr(m, name))
    counts = dict(
        nq=m.nq,
        nv=m.nv,
        nmocap=m.nmocap,
        nbody=m.nbody,
        njnt=m.njnt,
        ngeom=m.ngeom,
        nsite=m.nsite,
    )
    _validate_lowered(counts, values)
    return ModelDescriptor(**counts, **values)
