# Copyright 2025 The Newton Developers
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

from typing import Any, Tuple

import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src import math
from mujoco.mjx.third_party.mujoco_warp._src import ray
from mujoco.mjx.third_party.mujoco_warp._src import smooth
from mujoco.mjx.third_party.mujoco_warp._src import support
from mujoco.mjx.third_party.mujoco_warp._src.collision_sdf import sdf
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MAXCONPAIR
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MINVAL
from mujoco.mjx.third_party.mujoco_warp._src.types import ConeType
from mujoco.mjx.third_party.mujoco_warp._src.types import ConstraintType
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import DataType
from mujoco.mjx.third_party.mujoco_warp._src.types import DisableBit
from mujoco.mjx.third_party.mujoco_warp._src.types import GeomType
from mujoco.mjx.third_party.mujoco_warp._src.types import JointType
from mujoco.mjx.third_party.mujoco_warp._src.types import Model
from mujoco.mjx.third_party.mujoco_warp._src.types import ObjType
from mujoco.mjx.third_party.mujoco_warp._src.types import SensorType
from mujoco.mjx.third_party.mujoco_warp._src.types import TrnType
from mujoco.mjx.third_party.mujoco_warp._src.types import vec5
from mujoco.mjx.third_party.mujoco_warp._src.types import vec6
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import cache_kernel
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import kernel as nested_kernel


@wp.func
def _write_scalar(
  # Model:
  sensor_datatype: wp.array(dtype=int),
  sensor_adr: wp.array(dtype=int),
  sensor_cutoff: wp.array(dtype=float),
  # In:
  sensorid: int,
  sensor: Any,
  # Out:
  out: wp.array(dtype=float),
):
  adr = sensor_adr[sensorid]
  cutoff = sensor_cutoff[sensorid]

  if cutoff > 0.0:
    datatype = sensor_datatype[sensorid]
    if datatype == int(DataType.REAL.value):
      out[adr] = wp.clamp(sensor, -cutoff, cutoff)
    elif datatype == int(DataType.POSITIVE.value):
      out[adr] = wp.min(sensor, cutoff)
  else:
    out[adr] = sensor


@wp.func
def _write_vector(
  # Model:
  sensor_datatype: wp.array(dtype=int),
  sensor_adr: wp.array(dtype=int),
  sensor_cutoff: wp.array(dtype=float),
  # In:
  sensorid: int,
  sensordim: int,
  sensor: Any,
  # Out:
  out: wp.array(dtype=float),
):
  adr = sensor_adr[sensorid]
  cutoff = sensor_cutoff[sensorid]

  if cutoff > 0.0:
    datatype = sensor_datatype[sensorid]
    if datatype == int(DataType.REAL.value):
      for i in range(sensordim):
        out[adr + i] = wp.clamp(sensor[i], -cutoff, cutoff)
    elif datatype == int(DataType.POSITIVE.value):
      for i in range(sensordim):
        out[adr + i] = wp.min(sensor[i], cutoff)
  else:
    for i in range(sensordim):
      out[adr + i] = sensor[i]


@wp.func
def _magnetometer(
  # Model:
  opt_magnetic: wp.array(dtype=wp.vec3),
  # Data in:
  site_xmat_in: wp.array2d(dtype=wp.mat33),
  # In:
  worldid: int,
  objid: int,
) -> wp.vec3:
  magnetic = opt_magnetic[worldid]
  return wp.transpose(site_xmat_in[worldid, objid]) @ magnetic


@wp.func
def _cam_projection(
  # Model:
  cam_fovy: wp.array(dtype=float),
  cam_resolution: wp.array(dtype=wp.vec2i),
  cam_sensorsize: wp.array(dtype=wp.vec2),
  cam_intrinsic: wp.array(dtype=wp.vec4),
  # Data in:
  site_xpos_in: wp.array2d(dtype=wp.vec3),
  cam_xpos_in: wp.array2d(dtype=wp.vec3),
  cam_xmat_in: wp.array2d(dtype=wp.mat33),
  # In:
  worldid: int,
  objid: int,
  refid: int,
) -> wp.vec2:
  sensorsize = cam_sensorsize[refid]
  intrinsic = cam_intrinsic[refid]
  fovy = cam_fovy[refid]
  res = cam_resolution[refid]

  target_xpos = site_xpos_in[worldid, objid]
  xpos = cam_xpos_in[worldid, refid]
  xmat = cam_xmat_in[worldid, refid]

  translation = wp.mat44f(1.0, 0.0, 0.0, -xpos[0], 0.0, 1.0, 0.0, -xpos[1], 0.0, 0.0, 1.0, -xpos[2], 0.0, 0.0, 0.0, 1.0)
  rotation = wp.mat44f(
    xmat[0, 0], xmat[1, 0], xmat[2, 0], 0.0,
    xmat[0, 1], xmat[1, 1], xmat[2, 1], 0.0,
    xmat[0, 2], xmat[1, 2], xmat[2, 2], 0.0,
    0.0, 0.0, 0.0, 1.0,
  )  # fmt: skip

  # focal transformation matrix (3 x 4)
  if sensorsize[0] != 0.0 and sensorsize[1] != 0.0:
    fx = intrinsic[0] / (sensorsize[0] + MJ_MINVAL) * float(res[0])
    fy = intrinsic[1] / (sensorsize[1] + MJ_MINVAL) * float(res[1])
    focal = wp.mat44f(-fx, 0.0, 0.0, 0.0, 0.0, fy, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
  else:
    f = 0.5 / wp.tan(fovy * wp.static(wp.pi / 360.0)) * float(res[1])
    focal = wp.mat44f(-f, 0.0, 0.0, 0.0, 0.0, f, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0)

  # image matrix (3 x 3)
  image = wp.mat44f(
    1.0, 0.0, 0.5 * float(res[0]), 0.0, 0.0, 1.0, 0.5 * float(res[1]), 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0
  )

  # projection matrix (3 x 4): product of all 4 matrices
  # TODO(team): compute proj directly
  proj = image @ focal @ rotation @ translation

  # projection matrix multiples homogeneous [x, y, z, 1] vectors
  pos_hom = wp.vec4(target_xpos[0], target_xpos[1], target_xpos[2], 1.0)

  # project world coordinates into pixel space, see:
  # https://en.wikipedia.org/wiki/3D_projection#Mathematical_formula
  pixel_coord_hom = proj @ pos_hom

  # avoid dividing by tiny numbers
  denom = pixel_coord_hom[2]
  if wp.abs(denom) < MJ_MINVAL:
    denom = wp.clamp(denom, -MJ_MINVAL, MJ_MINVAL)

  # compute projection
  return wp.vec2f(pixel_coord_hom[0], pixel_coord_hom[1]) / denom


@wp.kernel
def _sensor_rangefinder_init(
  # Model:
  sensor_objid: wp.array(dtype=int),
  sensor_rangefinder_adr: wp.array(dtype=int),
  # Data in:
  site_xpos_in: wp.array2d(dtype=wp.vec3),
  site_xmat_in: wp.array2d(dtype=wp.mat33),
  # Data out:
  sensor_rangefinder_pnt_out: wp.array2d(dtype=wp.vec3),
  sensor_rangefinder_vec_out: wp.array2d(dtype=wp.vec3),
):
  worldid, rfid = wp.tid()
  sensorid = sensor_rangefinder_adr[rfid]
  objid = sensor_objid[sensorid]
  site_xpos = site_xpos_in[worldid, objid]
  site_xmat = site_xmat_in[worldid, objid]

  sensor_rangefinder_pnt_out[worldid, rfid] = site_xpos
  sensor_rangefinder_vec_out[worldid, rfid] = wp.vec3(site_xmat[0, 2], site_xmat[1, 2], site_xmat[2, 2])


@wp.func
def _joint_pos(jnt_qposadr: wp.array(dtype=int), qpos_in: wp.array2d(dtype=float), worldid: int, objid: int) -> float:
  return qpos_in[worldid, jnt_qposadr[objid]]


@wp.func
def _tendon_pos(ten_length_in: wp.array2d(dtype=float), worldid: int, objid: int) -> float:
  return ten_length_in[worldid, objid]


@wp.func
def _actuator_pos(actuator_length_in: wp.array2d(dtype=float), worldid: int, objid: int) -> float:
  return actuator_length_in[worldid, objid]


@wp.func
def _ball_quat(jnt_qposadr: wp.array(dtype=int), qpos_in: wp.array2d(dtype=float), worldid: int, objid: int) -> wp.quat:
  adr = jnt_qposadr[objid]
  quat = wp.quat(
    qpos_in[worldid, adr + 0],
    qpos_in[worldid, adr + 1],
    qpos_in[worldid, adr + 2],
    qpos_in[worldid, adr + 3],
  )
  return wp.normalize(quat)


@wp.kernel
def _limit_pos_zero(
  # Model:
  sensor_adr: wp.array(dtype=int),
  sensor_limitpos_adr: wp.array(dtype=int),
  # Data out:
  sensordata_out: wp.array2d(dtype=float),
):
  worldid, limitposid = wp.tid()
  sensordata_out[worldid, sensor_adr[sensor_limitpos_adr[limitposid]]] = 0.0


@wp.kernel
def _limit_pos(
  # Model:
  sensor_datatype: wp.array(dtype=int),
  sensor_objid: wp.array(dtype=int),
  sensor_adr: wp.array(dtype=int),
  sensor_cutoff: wp.array(dtype=float),
  sensor_limitpos_adr: wp.array(dtype=int),
  # Data in:
  ne_in: wp.array(dtype=int),
  nf_in: wp.array(dtype=int),
  nl_in: wp.array(dtype=int),
  efc_type_in: wp.array2d(dtype=int),
  efc_id_in: wp.array2d(dtype=int),
  efc_pos_in: wp.array2d(dtype=float),
  efc_margin_in: wp.array2d(dtype=float),
  # Data out:
  sensordata_out: wp.array2d(dtype=float),
):
  worldid, efcid, limitposid = wp.tid()

  ne = ne_in[worldid]
  nf = nf_in[worldid]
  nl = nl_in[worldid]

  # skip if not limit
  if efcid < ne + nf or efcid >= ne + nf + nl:
    return

  sensorid = sensor_limitpos_adr[limitposid]
  if efc_id_in[worldid, efcid] == sensor_objid[sensorid]:
    efc_type = efc_type_in[worldid, efcid]
    if efc_type == int(ConstraintType.LIMIT_JOINT.value) or efc_type == int(ConstraintType.LIMIT_TENDON.value):
      val = efc_pos_in[worldid, efcid] - efc_margin_in[worldid, efcid]
      _write_scalar(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, val, sensordata_out[worldid])


@wp.func
def _frame_pos(
  # Data in:
  xpos_in: wp.array2d(dtype=wp.vec3),
  xmat_in: wp.array2d(dtype=wp.mat33),
  xipos_in: wp.array2d(dtype=wp.vec3),
  ximat_in: wp.array2d(dtype=wp.mat33),
  geom_xpos_in: wp.array2d(dtype=wp.vec3),
  geom_xmat_in: wp.array2d(dtype=wp.mat33),
  site_xpos_in: wp.array2d(dtype=wp.vec3),
  site_xmat_in: wp.array2d(dtype=wp.mat33),
  cam_xpos_in: wp.array2d(dtype=wp.vec3),
  cam_xmat_in: wp.array2d(dtype=wp.mat33),
  # In:
  worldid: int,
  objid: int,
  objtype: int,
  refid: int,
  reftype: int,
) -> wp.vec3:
  if objtype == int(ObjType.BODY.value):
    xpos = xipos_in[worldid, objid]
  elif objtype == int(ObjType.XBODY.value):
    xpos = xpos_in[worldid, objid]
  elif objtype == int(ObjType.GEOM.value):
    xpos = geom_xpos_in[worldid, objid]
  elif objtype == int(ObjType.SITE.value):
    xpos = site_xpos_in[worldid, objid]
  elif objtype == int(ObjType.CAMERA.value):
    xpos = cam_xpos_in[worldid, objid]
  else:  # UNKNOWN
    xpos = wp.vec3(0.0)

  if refid == -1:
    return xpos

  if reftype == int(ObjType.BODY.value):
    xpos_ref = xipos_in[worldid, refid]
    xmat_ref = ximat_in[worldid, refid]
  elif objtype == int(ObjType.XBODY.value):
    xpos_ref = xpos_in[worldid, refid]
    xmat_ref = xmat_in[worldid, refid]
  elif reftype == int(ObjType.GEOM.value):
    xpos_ref = geom_xpos_in[worldid, refid]
    xmat_ref = geom_xmat_in[worldid, refid]
  elif reftype == int(ObjType.SITE.value):
    xpos_ref = site_xpos_in[worldid, refid]
    xmat_ref = site_xmat_in[worldid, refid]
  elif reftype == int(ObjType.CAMERA.value):
    xpos_ref = cam_xpos_in[worldid, refid]
    xmat_ref = cam_xmat_in[worldid, refid]

  else:  # UNKNOWN
    xpos_ref = wp.vec3(0.0)
    xmat_ref = wp.identity(3, wp.float32)

  return wp.transpose(xmat_ref) @ (xpos - xpos_ref)


@wp.func
def _frame_axis(
  # Data in:
  xmat_in: wp.array2d(dtype=wp.mat33),
  ximat_in: wp.array2d(dtype=wp.mat33),
  geom_xmat_in: wp.array2d(dtype=wp.mat33),
  site_xmat_in: wp.array2d(dtype=wp.mat33),
  cam_xmat_in: wp.array2d(dtype=wp.mat33),
  # In:
  worldid: int,
  objid: int,
  objtype: int,
  refid: int,
  reftype: int,
  frame_axis: int,
) -> wp.vec3:
  if objtype == int(ObjType.BODY.value):
    xmat = ximat_in[worldid, objid]
    axis = wp.vec3(xmat[0, frame_axis], xmat[1, frame_axis], xmat[2, frame_axis])
  elif objtype == int(ObjType.XBODY.value):
    xmat = xmat_in[worldid, objid]
    axis = wp.vec3(xmat[0, frame_axis], xmat[1, frame_axis], xmat[2, frame_axis])
  elif objtype == int(ObjType.GEOM.value):
    xmat = geom_xmat_in[worldid, objid]
    axis = wp.vec3(xmat[0, frame_axis], xmat[1, frame_axis], xmat[2, frame_axis])
  elif objtype == int(ObjType.SITE.value):
    xmat = site_xmat_in[worldid, objid]
    axis = wp.vec3(xmat[0, frame_axis], xmat[1, frame_axis], xmat[2, frame_axis])
  elif objtype == int(ObjType.CAMERA.value):
    xmat = cam_xmat_in[worldid, objid]
    axis = wp.vec3(xmat[0, frame_axis], xmat[1, frame_axis], xmat[2, frame_axis])
  else:  # UNKNOWN
    axis = wp.vec3(xmat[0, frame_axis], xmat[1, frame_axis], xmat[2, frame_axis])

  if refid == -1:
    return axis

  if reftype == int(ObjType.BODY.value):
    xmat_ref = ximat_in[worldid, refid]
  elif reftype == int(ObjType.XBODY.value):
    xmat_ref = xmat_in[worldid, refid]
  elif reftype == int(ObjType.GEOM.value):
    xmat_ref = geom_xmat_in[worldid, refid]
  elif reftype == int(ObjType.SITE.value):
    xmat_ref = site_xmat_in[worldid, refid]
  elif reftype == int(ObjType.CAMERA.value):
    xmat_ref = cam_xmat_in[worldid, refid]
  else:  # UNKNOWN
    xmat_ref = wp.identity(3, dtype=wp.float32)

  return wp.transpose(xmat_ref) @ axis


@wp.func
def _frame_quat(
  # Model:
  body_iquat: wp.array2d(dtype=wp.quat),
  geom_bodyid: wp.array(dtype=int),
  geom_quat: wp.array2d(dtype=wp.quat),
  site_bodyid: wp.array(dtype=int),
  site_quat: wp.array2d(dtype=wp.quat),
  cam_bodyid: wp.array(dtype=int),
  cam_quat: wp.array2d(dtype=wp.quat),
  # Data in:
  xquat_in: wp.array2d(dtype=wp.quat),
  # In:
  worldid: int,
  objid: int,
  objtype: int,
  refid: int,
  reftype: int,
) -> wp.quat:
  if objtype == int(ObjType.BODY.value):
    quat = math.mul_quat(xquat_in[worldid, objid], body_iquat[worldid, objid])
  elif objtype == int(ObjType.XBODY.value):
    quat = xquat_in[worldid, objid]
  elif objtype == int(ObjType.GEOM.value):
    quat = math.mul_quat(xquat_in[worldid, geom_bodyid[objid]], geom_quat[worldid, objid])
  elif objtype == int(ObjType.SITE.value):
    quat = math.mul_quat(xquat_in[worldid, site_bodyid[objid]], site_quat[worldid, objid])
  elif objtype == int(ObjType.CAMERA.value):
    quat = math.mul_quat(xquat_in[worldid, cam_bodyid[objid]], cam_quat[worldid, objid])
  else:  # UNKNOWN
    quat = wp.quat(1.0, 0.0, 0.0, 0.0)

  if refid == -1:
    return quat

  if reftype == int(ObjType.BODY.value):
    refquat = math.mul_quat(xquat_in[worldid, refid], body_iquat[worldid, refid])
  elif reftype == int(ObjType.XBODY.value):
    refquat = xquat_in[worldid, refid]
  elif reftype == int(ObjType.GEOM.value):
    refquat = math.mul_quat(xquat_in[worldid, geom_bodyid[refid]], geom_quat[worldid, refid])
  elif reftype == int(ObjType.SITE.value):
    refquat = math.mul_quat(xquat_in[worldid, site_bodyid[refid]], site_quat[worldid, refid])
  elif reftype == int(ObjType.CAMERA.value):
    refquat = math.mul_quat(xquat_in[worldid, cam_bodyid[refid]], cam_quat[worldid, refid])
  else:  # UNKNOWN
    refquat = wp.quat(1.0, 0.0, 0.0, 0.0)

  return math.mul_quat(math.quat_inv(refquat), quat)


@wp.func
def _subtree_com(subtree_com_in: wp.array2d(dtype=wp.vec3), worldid: int, objid: int) -> wp.vec3:
  return subtree_com_in[worldid, objid]


@wp.func
def _clock(time_in: wp.array(dtype=float), worldid: int) -> float:
  return time_in[worldid]


@wp.kernel
def _sensor_pos(
  # Model:
  opt_magnetic: wp.array(dtype=wp.vec3),
  body_iquat: wp.array2d(dtype=wp.quat),
  jnt_qposadr: wp.array(dtype=int),
  geom_bodyid: wp.array(dtype=int),
  geom_quat: wp.array2d(dtype=wp.quat),
  site_bodyid: wp.array(dtype=int),
  site_quat: wp.array2d(dtype=wp.quat),
  cam_bodyid: wp.array(dtype=int),
  cam_quat: wp.array2d(dtype=wp.quat),
  cam_fovy: wp.array(dtype=float),
  cam_resolution: wp.array(dtype=wp.vec2i),
  cam_sensorsize: wp.array(dtype=wp.vec2),
  cam_intrinsic: wp.array(dtype=wp.vec4),
  sensor_type: wp.array(dtype=int),
  sensor_datatype: wp.array(dtype=int),
  sensor_objtype: wp.array(dtype=int),
  sensor_objid: wp.array(dtype=int),
  sensor_reftype: wp.array(dtype=int),
  sensor_refid: wp.array(dtype=int),
  sensor_adr: wp.array(dtype=int),
  sensor_cutoff: wp.array(dtype=float),
  sensor_pos_adr: wp.array(dtype=int),
  rangefinder_sensor_adr: wp.array(dtype=int),
  # Data in:
  time_in: wp.array(dtype=float),
  energy_in: wp.array(dtype=wp.vec2),
  qpos_in: wp.array2d(dtype=float),
  xpos_in: wp.array2d(dtype=wp.vec3),
  xquat_in: wp.array2d(dtype=wp.quat),
  xmat_in: wp.array2d(dtype=wp.mat33),
  xipos_in: wp.array2d(dtype=wp.vec3),
  ximat_in: wp.array2d(dtype=wp.mat33),
  geom_xpos_in: wp.array2d(dtype=wp.vec3),
  geom_xmat_in: wp.array2d(dtype=wp.mat33),
  site_xpos_in: wp.array2d(dtype=wp.vec3),
  site_xmat_in: wp.array2d(dtype=wp.mat33),
  cam_xpos_in: wp.array2d(dtype=wp.vec3),
  cam_xmat_in: wp.array2d(dtype=wp.mat33),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  actuator_length_in: wp.array2d(dtype=float),
  ten_length_in: wp.array2d(dtype=float),
  sensor_rangefinder_dist_in: wp.array2d(dtype=float),
  # Data out:
  sensordata_out: wp.array2d(dtype=float),
):
  worldid, posid = wp.tid()
  sensorid = sensor_pos_adr[posid]
  sensortype = sensor_type[sensorid]
  objid = sensor_objid[sensorid]
  out = sensordata_out[worldid]

  if sensortype == int(SensorType.MAGNETOMETER.value):
    vec3 = _magnetometer(opt_magnetic, site_xmat_in, worldid, objid)
    _write_vector(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, vec3, out)
  elif sensortype == int(SensorType.CAMPROJECTION.value):
    refid = sensor_refid[sensorid]
    vec2 = _cam_projection(
      cam_fovy, cam_resolution, cam_sensorsize, cam_intrinsic, site_xpos_in, cam_xpos_in, cam_xmat_in, worldid, objid, refid
    )
    _write_vector(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 2, vec2, out)
  elif sensortype == int(SensorType.RANGEFINDER.value):
    val = sensor_rangefinder_dist_in[worldid, rangefinder_sensor_adr[sensorid]]
    _write_scalar(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, val, out)
  elif sensortype == int(SensorType.JOINTPOS.value):
    val = _joint_pos(jnt_qposadr, qpos_in, worldid, objid)
    _write_scalar(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, val, out)
  elif sensortype == int(SensorType.TENDONPOS.value):
    val = _tendon_pos(ten_length_in, worldid, objid)
    _write_scalar(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, val, out)
  elif sensortype == int(SensorType.ACTUATORPOS.value):
    val = _actuator_pos(actuator_length_in, worldid, objid)
    _write_scalar(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, val, out)
  elif sensortype == int(SensorType.BALLQUAT.value):
    quat = _ball_quat(jnt_qposadr, qpos_in, worldid, objid)
    _write_vector(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 4, quat, out)
  elif sensortype == int(SensorType.FRAMEPOS.value):
    objtype = sensor_objtype[sensorid]
    refid = sensor_refid[sensorid]
    reftype = sensor_reftype[sensorid]
    vec3 = _frame_pos(
      xpos_in,
      xmat_in,
      xipos_in,
      ximat_in,
      geom_xpos_in,
      geom_xmat_in,
      site_xpos_in,
      site_xmat_in,
      cam_xpos_in,
      cam_xmat_in,
      worldid,
      objid,
      objtype,
      refid,
      reftype,
    )
    _write_vector(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, vec3, out)
  elif (
    sensortype == int(SensorType.FRAMEXAXIS.value)
    or sensortype == int(SensorType.FRAMEYAXIS.value)
    or sensortype == int(SensorType.FRAMEZAXIS.value)
  ):
    objtype = sensor_objtype[sensorid]
    refid = sensor_refid[sensorid]
    reftype = sensor_reftype[sensorid]
    if sensortype == int(SensorType.FRAMEXAXIS.value):
      axis = 0
    elif sensortype == int(SensorType.FRAMEYAXIS.value):
      axis = 1
    elif sensortype == int(SensorType.FRAMEZAXIS.value):
      axis = 2
    vec3 = _frame_axis(
      ximat_in, xmat_in, geom_xmat_in, site_xmat_in, cam_xmat_in, worldid, objid, objtype, refid, reftype, axis
    )
    _write_vector(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, vec3, out)
  elif sensortype == int(SensorType.FRAMEQUAT.value):
    objtype = sensor_objtype[sensorid]
    refid = sensor_refid[sensorid]
    reftype = sensor_reftype[sensorid]
    quat = _frame_quat(
      body_iquat,
      geom_bodyid,
      geom_quat,
      site_bodyid,
      site_quat,
      cam_bodyid,
      cam_quat,
      xquat_in,
      worldid,
      objid,
      objtype,
      refid,
      reftype,
    )
    _write_vector(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 4, quat, out)
  elif sensortype == int(SensorType.SUBTREECOM.value):
    vec3 = _subtree_com(subtree_com_in, worldid, objid)
    _write_vector(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, vec3, out)
  elif sensortype == int(SensorType.E_POTENTIAL.value):
    val = energy_in[worldid][0]
    _write_scalar(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, val, out)
  elif sensortype == int(SensorType.E_KINETIC.value):
    val = energy_in[worldid][1]
    _write_scalar(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, val, out)
  elif sensortype == int(SensorType.CLOCK.value):
    val = _clock(time_in, worldid)
    _write_scalar(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, val, out)


@event_scope
def sensor_pos(m: Model, d: Data):
  """Compute position-dependent sensor values."""

  if m.opt.disableflags & DisableBit.SENSOR:
    return

  # rangefinder
  if m.sensor_rangefinder_adr.size > 0:
    # get position and direction
    wp.launch(
      _sensor_rangefinder_init,
      dim=(d.nworld, m.sensor_rangefinder_adr.size),
      inputs=[
        m.sensor_objid,
        m.sensor_rangefinder_adr,
        d.site_xpos,
        d.site_xmat,
      ],
      outputs=[
        d.sensor_rangefinder_pnt,
        d.sensor_rangefinder_vec,
      ],
    )

    # get distances
    ray.rays(
      m,
      d,
      d.sensor_rangefinder_pnt,
      d.sensor_rangefinder_vec,
      vec6(wp.inf, wp.inf, wp.inf, wp.inf, wp.inf, wp.inf),
      True,
      m.sensor_rangefinder_bodyid,
      d.sensor_rangefinder_dist,
      d.sensor_rangefinder_geomid,
    )

  if m.sensor_e_potential:
    energy_pos(m, d)

  if m.sensor_e_kinetic:
    energy_vel(m, d)

  wp.launch(
    _sensor_pos,
    dim=(d.nworld, m.sensor_pos_adr.size),
    inputs=[
      m.opt.magnetic,
      m.body_iquat,
      m.jnt_qposadr,
      m.geom_bodyid,
      m.geom_quat,
      m.site_bodyid,
      m.site_quat,
      m.cam_bodyid,
      m.cam_quat,
      m.cam_fovy,
      m.cam_resolution,
      m.cam_sensorsize,
      m.cam_intrinsic,
      m.sensor_type,
      m.sensor_datatype,
      m.sensor_objtype,
      m.sensor_objid,
      m.sensor_reftype,
      m.sensor_refid,
      m.sensor_adr,
      m.sensor_cutoff,
      m.sensor_pos_adr,
      m.rangefinder_sensor_adr,
      d.time,
      d.energy,
      d.qpos,
      d.xpos,
      d.xquat,
      d.xmat,
      d.xipos,
      d.ximat,
      d.geom_xpos,
      d.geom_xmat,
      d.site_xpos,
      d.site_xmat,
      d.cam_xpos,
      d.cam_xmat,
      d.subtree_com,
      d.actuator_length,
      d.ten_length,
      d.sensor_rangefinder_dist,
    ],
    outputs=[d.sensordata],
  )

  # jointlimitpos and tendonlimitpos
  wp.launch(
    _limit_pos_zero,
    dim=(d.nworld, m.sensor_limitpos_adr.size),
    inputs=[m.sensor_adr, m.sensor_limitpos_adr],
    outputs=[d.sensordata],
  )

  wp.launch(
    _limit_pos,
    dim=(d.nworld, d.njmax, m.sensor_limitpos_adr.size),
    inputs=[
      m.sensor_datatype,
      m.sensor_objid,
      m.sensor_adr,
      m.sensor_cutoff,
      m.sensor_limitpos_adr,
      d.ne,
      d.nf,
      d.nl,
      d.efc.type,
      d.efc.id,
      d.efc.pos,
      d.efc.margin,
    ],
    outputs=[
      d.sensordata,
    ],
  )


@wp.func
def _velocimeter(
  # Model:
  body_rootid: wp.array(dtype=int),
  site_bodyid: wp.array(dtype=int),
  # Data in:
  site_xpos_in: wp.array2d(dtype=wp.vec3),
  site_xmat_in: wp.array2d(dtype=wp.mat33),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  cvel_in: wp.array2d(dtype=wp.spatial_vector),
  # In:
  worldid: int,
  objid: int,
) -> wp.vec3:
  bodyid = site_bodyid[objid]
  pos = site_xpos_in[worldid, objid]
  rot = site_xmat_in[worldid, objid]
  cvel = cvel_in[worldid, bodyid]
  ang = wp.spatial_top(cvel)
  lin = wp.spatial_bottom(cvel)
  subtree_com = subtree_com_in[worldid, body_rootid[bodyid]]
  dif = pos - subtree_com
  return wp.transpose(rot) @ (lin - wp.cross(dif, ang))


@wp.func
def _gyro(
  # Model:
  site_bodyid: wp.array(dtype=int),
  # Data in:
  site_xmat_in: wp.array2d(dtype=wp.mat33),
  cvel_in: wp.array2d(dtype=wp.spatial_vector),
  # In:
  worldid: int,
  objid: int,
) -> wp.vec3:
  bodyid = site_bodyid[objid]
  rot = site_xmat_in[worldid, objid]
  cvel = cvel_in[worldid, bodyid]
  ang = wp.spatial_top(cvel)
  return wp.transpose(rot) @ ang


@wp.func
def _joint_vel(jnt_dofadr: wp.array(dtype=int), qvel_in: wp.array2d(dtype=float), worldid: int, objid: int) -> float:
  return qvel_in[worldid, jnt_dofadr[objid]]


@wp.func
def _tendon_vel(ten_velocity_in: wp.array2d(dtype=float), worldid: int, objid: int) -> float:
  return ten_velocity_in[worldid, objid]


@wp.func
def _actuator_vel(actuator_velocity_in: wp.array2d(dtype=float), worldid: int, objid: int) -> float:
  return actuator_velocity_in[worldid, objid]


@wp.func
def _ball_ang_vel(jnt_dofadr: wp.array(dtype=int), qvel_in: wp.array2d(dtype=float), worldid: int, objid: int) -> wp.vec3:
  adr = jnt_dofadr[objid]
  return wp.vec3(qvel_in[worldid, adr + 0], qvel_in[worldid, adr + 1], qvel_in[worldid, adr + 2])


@wp.kernel
def _limit_vel_zero(
  # Model:
  sensor_adr: wp.array(dtype=int),
  sensor_limitvel_adr: wp.array(dtype=int),
  # Data out:
  sensordata_out: wp.array2d(dtype=float),
):
  worldid, limitvelid = wp.tid()
  sensordata_out[worldid, sensor_adr[sensor_limitvel_adr[limitvelid]]] = 0.0


@wp.kernel
def _limit_vel(
  # Model:
  sensor_datatype: wp.array(dtype=int),
  sensor_objid: wp.array(dtype=int),
  sensor_adr: wp.array(dtype=int),
  sensor_cutoff: wp.array(dtype=float),
  sensor_limitvel_adr: wp.array(dtype=int),
  # Data in:
  ne_in: wp.array(dtype=int),
  nf_in: wp.array(dtype=int),
  nl_in: wp.array(dtype=int),
  efc_type_in: wp.array2d(dtype=int),
  efc_id_in: wp.array2d(dtype=int),
  efc_vel_in: wp.array2d(dtype=float),
  # Data out:
  sensordata_out: wp.array2d(dtype=float),
):
  worldid, efcid, limitvelid = wp.tid()

  ne = ne_in[worldid]
  nf = nf_in[worldid]
  nl = nl_in[worldid]

  # skip if not limit
  if efcid < ne + nf or efcid >= ne + nf + nl:
    return

  sensorid = sensor_limitvel_adr[limitvelid]
  if efc_id_in[worldid, efcid] == sensor_objid[sensorid]:
    efc_type = efc_type_in[worldid, efcid]
    if efc_type == int(ConstraintType.LIMIT_JOINT.value) or efc_type == int(ConstraintType.LIMIT_TENDON.value):
      _write_scalar(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, efc_vel_in[worldid, efcid], sensordata_out[worldid])


@wp.func
def _cvel_offset(
  # Model:
  body_rootid: wp.array(dtype=int),
  geom_bodyid: wp.array(dtype=int),
  site_bodyid: wp.array(dtype=int),
  cam_bodyid: wp.array(dtype=int),
  # Data in:
  xpos_in: wp.array2d(dtype=wp.vec3),
  xipos_in: wp.array2d(dtype=wp.vec3),
  geom_xpos_in: wp.array2d(dtype=wp.vec3),
  site_xpos_in: wp.array2d(dtype=wp.vec3),
  cam_xpos_in: wp.array2d(dtype=wp.vec3),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  cvel_in: wp.array2d(dtype=wp.spatial_vector),
  # In:
  worldid: int,
  objtype: int,
  objid: int,
) -> Tuple[wp.spatial_vector, wp.vec3]:
  if objtype == int(ObjType.BODY.value):
    pos = xipos_in[worldid, objid]
    bodyid = objid
  elif objtype == int(ObjType.XBODY.value):
    pos = xpos_in[worldid, objid]
    bodyid = objid
  elif objtype == int(ObjType.GEOM.value):
    pos = geom_xpos_in[worldid, objid]
    bodyid = geom_bodyid[objid]
  elif objtype == int(ObjType.SITE.value):
    pos = site_xpos_in[worldid, objid]
    bodyid = site_bodyid[objid]
  elif objtype == int(ObjType.CAMERA.value):
    pos = cam_xpos_in[worldid, objid]
    bodyid = cam_bodyid[objid]
  else:  # UNKNOWN
    pos = wp.vec3(0.0)
    bodyid = 0

  return cvel_in[worldid, bodyid], pos - subtree_com_in[worldid, body_rootid[bodyid]]


@wp.func
def _frame_linvel(
  # Model:
  body_rootid: wp.array(dtype=int),
  geom_bodyid: wp.array(dtype=int),
  site_bodyid: wp.array(dtype=int),
  cam_bodyid: wp.array(dtype=int),
  # Data in:
  xpos_in: wp.array2d(dtype=wp.vec3),
  xmat_in: wp.array2d(dtype=wp.mat33),
  xipos_in: wp.array2d(dtype=wp.vec3),
  ximat_in: wp.array2d(dtype=wp.mat33),
  geom_xpos_in: wp.array2d(dtype=wp.vec3),
  geom_xmat_in: wp.array2d(dtype=wp.mat33),
  site_xpos_in: wp.array2d(dtype=wp.vec3),
  site_xmat_in: wp.array2d(dtype=wp.mat33),
  cam_xpos_in: wp.array2d(dtype=wp.vec3),
  cam_xmat_in: wp.array2d(dtype=wp.mat33),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  cvel_in: wp.array2d(dtype=wp.spatial_vector),
  # In:
  worldid: int,
  objid: int,
  objtype: int,
  refid: int,
  reftype: int,
) -> wp.vec3:
  if objtype == int(ObjType.BODY.value):
    xpos = xipos_in[worldid, objid]
  elif objtype == int(ObjType.XBODY.value):
    xpos = xpos_in[worldid, objid]
  elif objtype == int(ObjType.GEOM.value):
    xpos = geom_xpos_in[worldid, objid]
  elif objtype == int(ObjType.SITE.value):
    xpos = site_xpos_in[worldid, objid]
  elif objtype == int(ObjType.CAMERA.value):
    xpos = cam_xpos_in[worldid, objid]
  else:  # UNKNOWN
    xpos = wp.vec3(0.0)

  if reftype == int(ObjType.BODY.value):
    xposref = xipos_in[worldid, refid]
    xmatref = ximat_in[worldid, refid]
  elif reftype == int(ObjType.XBODY.value):
    xposref = xpos_in[worldid, refid]
    xmatref = xmat_in[worldid, refid]
  elif reftype == int(ObjType.GEOM.value):
    xposref = geom_xpos_in[worldid, refid]
    xmatref = geom_xmat_in[worldid, refid]
  elif reftype == int(ObjType.SITE.value):
    xposref = site_xpos_in[worldid, refid]
    xmatref = site_xmat_in[worldid, refid]
  elif reftype == int(ObjType.CAMERA.value):
    xposref = cam_xpos_in[worldid, refid]
    xmatref = cam_xmat_in[worldid, refid]
  else:  # UNKNOWN
    xposref = wp.vec3(0.0)
    xmatref = wp.identity(3, dtype=float)

  cvel, offset = _cvel_offset(
    body_rootid,
    geom_bodyid,
    site_bodyid,
    cam_bodyid,
    xpos_in,
    xipos_in,
    geom_xpos_in,
    site_xpos_in,
    cam_xpos_in,
    subtree_com_in,
    cvel_in,
    worldid,
    objtype,
    objid,
  )
  cvelref, offsetref = _cvel_offset(
    body_rootid,
    geom_bodyid,
    site_bodyid,
    cam_bodyid,
    xpos_in,
    xipos_in,
    geom_xpos_in,
    site_xpos_in,
    cam_xpos_in,
    subtree_com_in,
    cvel_in,
    worldid,
    reftype,
    refid,
  )
  clinvel = wp.spatial_bottom(cvel)
  cangvel = wp.spatial_top(cvel)
  cangvelref = wp.spatial_top(cvelref)
  xlinvel = clinvel - wp.cross(offset, cangvel)

  if refid > -1:
    clinvelref = wp.spatial_bottom(cvelref)
    xlinvelref = clinvelref - wp.cross(offsetref, cangvelref)
    rvec = xpos - xposref
    rel_vel = xlinvel - xlinvelref + wp.cross(rvec, cangvelref)
    return wp.transpose(xmatref) @ rel_vel
  else:
    return xlinvel


@wp.func
def _frame_angvel(
  # Model:
  body_rootid: wp.array(dtype=int),
  geom_bodyid: wp.array(dtype=int),
  site_bodyid: wp.array(dtype=int),
  cam_bodyid: wp.array(dtype=int),
  # Data in:
  xpos_in: wp.array2d(dtype=wp.vec3),
  xmat_in: wp.array2d(dtype=wp.mat33),
  xipos_in: wp.array2d(dtype=wp.vec3),
  ximat_in: wp.array2d(dtype=wp.mat33),
  geom_xpos_in: wp.array2d(dtype=wp.vec3),
  geom_xmat_in: wp.array2d(dtype=wp.mat33),
  site_xpos_in: wp.array2d(dtype=wp.vec3),
  site_xmat_in: wp.array2d(dtype=wp.mat33),
  cam_xpos_in: wp.array2d(dtype=wp.vec3),
  cam_xmat_in: wp.array2d(dtype=wp.mat33),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  cvel_in: wp.array2d(dtype=wp.spatial_vector),
  # In:
  worldid: int,
  objid: int,
  objtype: int,
  refid: int,
  reftype: int,
) -> wp.vec3:
  cvel, _ = _cvel_offset(
    body_rootid,
    geom_bodyid,
    site_bodyid,
    cam_bodyid,
    xpos_in,
    xipos_in,
    geom_xpos_in,
    site_xpos_in,
    cam_xpos_in,
    subtree_com_in,
    cvel_in,
    worldid,
    objtype,
    objid,
  )
  cangvel = wp.spatial_top(cvel)

  if refid > -1:
    if reftype == int(ObjType.BODY.value):
      xmatref = ximat_in[worldid, refid]
    elif reftype == int(ObjType.XBODY.value):
      xmatref = xmat_in[worldid, refid]
    elif reftype == int(ObjType.GEOM.value):
      xmatref = geom_xmat_in[worldid, refid]
    elif reftype == int(ObjType.SITE.value):
      xmatref = site_xmat_in[worldid, refid]
    elif reftype == int(ObjType.CAMERA.value):
      xmatref = cam_xmat_in[worldid, refid]
    else:  # UNKNOWN
      xmatref = wp.identity(3, dtype=float)

    cvelref, _ = _cvel_offset(
      body_rootid,
      geom_bodyid,
      site_bodyid,
      cam_bodyid,
      xpos_in,
      xipos_in,
      geom_xpos_in,
      site_xpos_in,
      cam_xpos_in,
      subtree_com_in,
      cvel_in,
      worldid,
      reftype,
      refid,
    )
    cangvelref = wp.spatial_top(cvelref)

    return wp.transpose(xmatref) @ (cangvel - cangvelref)
  else:
    return cangvel


@wp.func
def _subtree_linvel(subtree_linvel_in: wp.array2d(dtype=wp.vec3), worldid: int, objid: int) -> wp.vec3:
  return subtree_linvel_in[worldid, objid]


@wp.func
def _subtree_angmom(subtree_angmom_in: wp.array2d(dtype=wp.vec3), worldid: int, objid: int) -> wp.vec3:
  return subtree_angmom_in[worldid, objid]


@wp.kernel
def _sensor_vel(
  # Model:
  body_rootid: wp.array(dtype=int),
  jnt_dofadr: wp.array(dtype=int),
  geom_bodyid: wp.array(dtype=int),
  site_bodyid: wp.array(dtype=int),
  cam_bodyid: wp.array(dtype=int),
  sensor_type: wp.array(dtype=int),
  sensor_datatype: wp.array(dtype=int),
  sensor_objtype: wp.array(dtype=int),
  sensor_objid: wp.array(dtype=int),
  sensor_reftype: wp.array(dtype=int),
  sensor_refid: wp.array(dtype=int),
  sensor_adr: wp.array(dtype=int),
  sensor_cutoff: wp.array(dtype=float),
  sensor_vel_adr: wp.array(dtype=int),
  # Data in:
  qvel_in: wp.array2d(dtype=float),
  xpos_in: wp.array2d(dtype=wp.vec3),
  xmat_in: wp.array2d(dtype=wp.mat33),
  xipos_in: wp.array2d(dtype=wp.vec3),
  ximat_in: wp.array2d(dtype=wp.mat33),
  geom_xpos_in: wp.array2d(dtype=wp.vec3),
  geom_xmat_in: wp.array2d(dtype=wp.mat33),
  site_xpos_in: wp.array2d(dtype=wp.vec3),
  site_xmat_in: wp.array2d(dtype=wp.mat33),
  cam_xpos_in: wp.array2d(dtype=wp.vec3),
  cam_xmat_in: wp.array2d(dtype=wp.mat33),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  ten_velocity_in: wp.array2d(dtype=float),
  actuator_velocity_in: wp.array2d(dtype=float),
  cvel_in: wp.array2d(dtype=wp.spatial_vector),
  subtree_linvel_in: wp.array2d(dtype=wp.vec3),
  subtree_angmom_in: wp.array2d(dtype=wp.vec3),
  # Data out:
  sensordata_out: wp.array2d(dtype=float),
):
  worldid, velid = wp.tid()
  sensorid = sensor_vel_adr[velid]
  sensortype = sensor_type[sensorid]
  objid = sensor_objid[sensorid]
  out = sensordata_out[worldid]

  if sensortype == int(SensorType.VELOCIMETER.value):
    vec3 = _velocimeter(body_rootid, site_bodyid, site_xpos_in, site_xmat_in, subtree_com_in, cvel_in, worldid, objid)
    _write_vector(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, vec3, out)
  elif sensortype == int(SensorType.GYRO.value):
    vec3 = _gyro(site_bodyid, site_xmat_in, cvel_in, worldid, objid)
    _write_vector(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, vec3, out)
  elif sensortype == int(SensorType.JOINTVEL.value):
    val = _joint_vel(jnt_dofadr, qvel_in, worldid, objid)
    _write_scalar(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, val, out)
  elif sensortype == int(SensorType.TENDONVEL.value):
    val = _tendon_vel(ten_velocity_in, worldid, objid)
    _write_scalar(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, val, out)
  elif sensortype == int(SensorType.ACTUATORVEL.value):
    val = _actuator_vel(actuator_velocity_in, worldid, objid)
    _write_scalar(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, val, out)
  elif sensortype == int(SensorType.BALLANGVEL.value):
    vec3 = _ball_ang_vel(jnt_dofadr, qvel_in, worldid, objid)
    _write_vector(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, vec3, out)
  elif sensortype == int(SensorType.FRAMELINVEL.value):
    objtype = sensor_objtype[sensorid]
    refid = sensor_refid[sensorid]
    reftype = sensor_reftype[sensorid]
    frame_linvel = _frame_linvel(
      body_rootid,
      geom_bodyid,
      site_bodyid,
      cam_bodyid,
      xpos_in,
      xmat_in,
      xipos_in,
      ximat_in,
      geom_xpos_in,
      geom_xmat_in,
      site_xpos_in,
      site_xmat_in,
      cam_xpos_in,
      cam_xmat_in,
      subtree_com_in,
      cvel_in,
      worldid,
      objid,
      objtype,
      refid,
      reftype,
    )
    _write_vector(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, frame_linvel, out)
  elif sensortype == int(SensorType.FRAMEANGVEL.value):
    objtype = sensor_objtype[sensorid]
    refid = sensor_refid[sensorid]
    reftype = sensor_reftype[sensorid]
    frame_angvel = _frame_angvel(
      body_rootid,
      geom_bodyid,
      site_bodyid,
      cam_bodyid,
      xpos_in,
      xmat_in,
      xipos_in,
      ximat_in,
      geom_xpos_in,
      geom_xmat_in,
      site_xpos_in,
      site_xmat_in,
      cam_xpos_in,
      cam_xmat_in,
      subtree_com_in,
      cvel_in,
      worldid,
      objid,
      objtype,
      refid,
      reftype,
    )
    _write_vector(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, frame_angvel, out)
  elif sensortype == int(SensorType.SUBTREELINVEL.value):
    vec3 = _subtree_linvel(subtree_linvel_in, worldid, objid)
    _write_vector(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, vec3, out)
  elif sensortype == int(SensorType.SUBTREEANGMOM.value):
    vec3 = _subtree_angmom(subtree_angmom_in, worldid, objid)
    _write_vector(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, vec3, out)


@event_scope
def sensor_vel(m: Model, d: Data):
  """Compute velocity-dependent sensor values."""

  if m.opt.disableflags & DisableBit.SENSOR:
    return

  if m.sensor_subtree_vel:
    smooth.subtree_vel(m, d)

  wp.launch(
    _sensor_vel,
    dim=(d.nworld, m.sensor_vel_adr.size),
    inputs=[
      m.body_rootid,
      m.jnt_dofadr,
      m.geom_bodyid,
      m.site_bodyid,
      m.cam_bodyid,
      m.sensor_type,
      m.sensor_datatype,
      m.sensor_objtype,
      m.sensor_objid,
      m.sensor_reftype,
      m.sensor_refid,
      m.sensor_adr,
      m.sensor_cutoff,
      m.sensor_vel_adr,
      d.qvel,
      d.xpos,
      d.xmat,
      d.xipos,
      d.ximat,
      d.geom_xpos,
      d.geom_xmat,
      d.site_xpos,
      d.site_xmat,
      d.cam_xpos,
      d.cam_xmat,
      d.subtree_com,
      d.ten_velocity,
      d.actuator_velocity,
      d.cvel,
      d.subtree_linvel,
      d.subtree_angmom,
    ],
    outputs=[d.sensordata],
  )

  wp.launch(
    _limit_vel_zero,
    dim=(d.nworld, m.sensor_limitvel_adr.size),
    inputs=[m.sensor_adr, m.sensor_limitvel_adr],
    outputs=[d.sensordata],
  )

  wp.launch(
    _limit_vel,
    dim=(d.nworld, d.njmax, m.sensor_limitvel_adr.size),
    inputs=[
      m.sensor_datatype,
      m.sensor_objid,
      m.sensor_adr,
      m.sensor_cutoff,
      m.sensor_limitvel_adr,
      d.ne,
      d.nf,
      d.nl,
      d.efc.type,
      d.efc.id,
      d.efc.vel,
    ],
    outputs=[
      d.sensordata,
    ],
  )


@wp.func
def _accelerometer(
  # Model:
  body_rootid: wp.array(dtype=int),
  site_bodyid: wp.array(dtype=int),
  # Data in:
  site_xpos_in: wp.array2d(dtype=wp.vec3),
  site_xmat_in: wp.array2d(dtype=wp.mat33),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  cvel_in: wp.array2d(dtype=wp.spatial_vector),
  cacc_in: wp.array2d(dtype=wp.spatial_vector),
  # In:
  worldid: int,
  objid: int,
) -> wp.vec3:
  bodyid = site_bodyid[objid]
  rot = site_xmat_in[worldid, objid]
  rotT = wp.transpose(rot)
  cvel = cvel_in[worldid, bodyid]
  cvel_top = wp.spatial_top(cvel)
  cvel_bottom = wp.spatial_bottom(cvel)
  cacc = cacc_in[worldid, bodyid]
  cacc_top = wp.spatial_top(cacc)
  cacc_bottom = wp.spatial_bottom(cacc)
  dif = site_xpos_in[worldid, objid] - subtree_com_in[worldid, body_rootid[bodyid]]
  ang = rotT @ cvel_top
  lin = rotT @ (cvel_bottom - wp.cross(dif, cvel_top))
  acc = rotT @ (cacc_bottom - wp.cross(dif, cacc_top))
  correction = wp.cross(ang, lin)
  return acc + correction


@wp.func
def _force(
  # Model:
  site_bodyid: wp.array(dtype=int),
  # Data in:
  site_xmat_in: wp.array2d(dtype=wp.mat33),
  cfrc_int_in: wp.array2d(dtype=wp.spatial_vector),
  # In:
  worldid: int,
  objid: int,
) -> wp.vec3:
  bodyid = site_bodyid[objid]
  cfrc_int = cfrc_int_in[worldid, bodyid]
  site_xmat = site_xmat_in[worldid, objid]
  return wp.transpose(site_xmat) @ wp.spatial_bottom(cfrc_int)


@wp.func
def _torque(
  # Model:
  body_rootid: wp.array(dtype=int),
  site_bodyid: wp.array(dtype=int),
  # Data in:
  site_xpos_in: wp.array2d(dtype=wp.vec3),
  site_xmat_in: wp.array2d(dtype=wp.mat33),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  cfrc_int_in: wp.array2d(dtype=wp.spatial_vector),
  # In:
  worldid: int,
  objid: int,
) -> wp.vec3:
  bodyid = site_bodyid[objid]
  cfrc_int = cfrc_int_in[worldid, bodyid]
  site_xmat = site_xmat_in[worldid, objid]
  dif = site_xpos_in[worldid, objid] - subtree_com_in[worldid, body_rootid[bodyid]]
  return wp.transpose(site_xmat) @ (wp.spatial_top(cfrc_int) - wp.cross(dif, wp.spatial_bottom(cfrc_int)))


@wp.func
def _actuator_force(actuator_force_in: wp.array2d(dtype=float), worldid: int, objid: int) -> float:
  return actuator_force_in[worldid, objid]


@wp.func
def _joint_actuator_force(
  # Model:
  jnt_dofadr: wp.array(dtype=int),
  # Data in:
  qfrc_actuator_in: wp.array2d(dtype=float),
  # In:
  worldid: int,
  objid: int,
) -> float:
  return qfrc_actuator_in[worldid, jnt_dofadr[objid]]


@wp.kernel
def _tendon_actuator_force_zero(
  # Model:
  sensor_adr: wp.array(dtype=int),
  sensor_tendonactfrc_adr: wp.array(dtype=int),
  # Data out:
  sensordata_out: wp.array2d(dtype=float),
):
  worldid, tenactfrcid = wp.tid()
  sensorid = sensor_tendonactfrc_adr[tenactfrcid]
  adr = sensor_adr[sensorid]
  sensordata_out[worldid, adr] = 0.0


@wp.kernel
def _tendon_actuator_force(
  # Model:
  actuator_trntype: wp.array(dtype=int),
  actuator_trnid: wp.array(dtype=wp.vec2i),
  sensor_objid: wp.array(dtype=int),
  sensor_adr: wp.array(dtype=int),
  sensor_tendonactfrc_adr: wp.array(dtype=int),
  # Data in:
  actuator_force_in: wp.array2d(dtype=float),
  # Data out:
  sensordata_out: wp.array2d(dtype=float),
):
  worldid, tenactfrcid, actid = wp.tid()
  sensorid = sensor_tendonactfrc_adr[tenactfrcid]

  if actuator_trntype[actid] == int(TrnType.TENDON.value) and actuator_trnid[actid][0] == sensor_objid[sensorid]:
    adr = sensor_adr[sensorid]
    sensordata_out[worldid, adr] += actuator_force_in[worldid, actid]


@wp.kernel
def _tendon_actuator_force_cutoff(
  # Model:
  sensor_datatype: wp.array(dtype=int),
  sensor_adr: wp.array(dtype=int),
  sensor_cutoff: wp.array(dtype=float),
  sensor_tendonactfrc_adr: wp.array(dtype=int),
  # Data in:
  sensordata_in: wp.array2d(dtype=float),
  # Data out:
  sensordata_out: wp.array2d(dtype=float),
):
  worldid, tenactfrcid = wp.tid()
  sensorid = sensor_tendonactfrc_adr[tenactfrcid]
  adr = sensor_adr[sensorid]
  val = sensordata_in[worldid, adr]

  _write_scalar(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, val, sensordata_out[worldid])


@wp.kernel
def _limit_frc_zero(
  # Model:
  sensor_adr: wp.array(dtype=int),
  sensor_limitfrc_adr: wp.array(dtype=int),
  # Data out:
  sensordata_out: wp.array2d(dtype=float),
):
  worldid, limitfrcid = wp.tid()
  sensordata_out[worldid, sensor_adr[sensor_limitfrc_adr[limitfrcid]]] = 0.0


@wp.kernel
def _limit_frc(
  # Model:
  sensor_datatype: wp.array(dtype=int),
  sensor_objid: wp.array(dtype=int),
  sensor_adr: wp.array(dtype=int),
  sensor_cutoff: wp.array(dtype=float),
  sensor_limitfrc_adr: wp.array(dtype=int),
  # Data in:
  ne_in: wp.array(dtype=int),
  nf_in: wp.array(dtype=int),
  nl_in: wp.array(dtype=int),
  efc_type_in: wp.array2d(dtype=int),
  efc_id_in: wp.array2d(dtype=int),
  efc_force_in: wp.array2d(dtype=float),
  # Data out:
  sensordata_out: wp.array2d(dtype=float),
):
  worldid, efcid, limitfrcid = wp.tid()

  ne = ne_in[worldid]
  nf = nf_in[worldid]
  nl = nl_in[worldid]

  # skip if not limit
  if efcid < ne + nf or efcid >= ne + nf + nl:
    return

  sensorid = sensor_limitfrc_adr[limitfrcid]
  if efc_id_in[worldid, efcid] == sensor_objid[sensorid]:
    efc_type = efc_type_in[worldid, efcid]
    if efc_type == int(ConstraintType.LIMIT_JOINT.value) or efc_type == int(ConstraintType.LIMIT_TENDON.value):
      _write_scalar(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, efc_force_in[worldid, efcid], sensordata_out[worldid])


@wp.func
def _framelinacc(
  # Model:
  body_rootid: wp.array(dtype=int),
  geom_bodyid: wp.array(dtype=int),
  site_bodyid: wp.array(dtype=int),
  cam_bodyid: wp.array(dtype=int),
  # Data in:
  xpos_in: wp.array2d(dtype=wp.vec3),
  xipos_in: wp.array2d(dtype=wp.vec3),
  geom_xpos_in: wp.array2d(dtype=wp.vec3),
  site_xpos_in: wp.array2d(dtype=wp.vec3),
  cam_xpos_in: wp.array2d(dtype=wp.vec3),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  cvel_in: wp.array2d(dtype=wp.spatial_vector),
  cacc_in: wp.array2d(dtype=wp.spatial_vector),
  # In:
  worldid: int,
  objid: int,
  objtype: int,
) -> wp.vec3:
  if objtype == int(ObjType.BODY.value):
    bodyid = objid
    pos = xipos_in[worldid, objid]
  elif objtype == int(ObjType.XBODY.value):
    bodyid = objid
    pos = xpos_in[worldid, objid]
  elif objtype == int(ObjType.GEOM.value):
    bodyid = geom_bodyid[objid]
    pos = geom_xpos_in[worldid, objid]
  elif objtype == int(ObjType.SITE.value):
    bodyid = site_bodyid[objid]
    pos = site_xpos_in[worldid, objid]
  elif objtype == int(ObjType.CAMERA.value):
    bodyid = cam_bodyid[objid]
    pos = cam_xpos_in[worldid, objid]
  else:  # UNKNOWN
    bodyid = 0
    pos = wp.vec3(0.0)

  cacc = cacc_in[worldid, bodyid]
  cvel = cvel_in[worldid, bodyid]
  offset = pos - subtree_com_in[worldid, body_rootid[bodyid]]
  ang = wp.spatial_top(cvel)
  lin = wp.spatial_bottom(cvel) - wp.cross(offset, ang)
  acc = wp.spatial_bottom(cacc) - wp.cross(offset, wp.spatial_top(cacc))
  correction = wp.cross(ang, lin)

  return acc + correction


@wp.func
def _frameangacc(
  # Model:
  geom_bodyid: wp.array(dtype=int),
  site_bodyid: wp.array(dtype=int),
  cam_bodyid: wp.array(dtype=int),
  # Data in:
  cacc_in: wp.array2d(dtype=wp.spatial_vector),
  # In:
  worldid: int,
  objid: int,
  objtype: int,
) -> wp.vec3:
  if objtype == int(ObjType.BODY.value) or objtype == int(ObjType.XBODY.value):
    bodyid = objid
  elif objtype == int(ObjType.GEOM.value):
    bodyid = geom_bodyid[objid]
  elif objtype == int(ObjType.SITE.value):
    bodyid = site_bodyid[objid]
  elif objtype == int(ObjType.CAMERA.value):
    bodyid = cam_bodyid[objid]
  else:  # UNKNOWN
    bodyid = 0

  return wp.spatial_top(cacc_in[worldid, bodyid])


@wp.kernel
def _sensor_acc(
  # Model:
  opt_cone: int,
  body_rootid: wp.array(dtype=int),
  jnt_dofadr: wp.array(dtype=int),
  geom_bodyid: wp.array(dtype=int),
  site_bodyid: wp.array(dtype=int),
  cam_bodyid: wp.array(dtype=int),
  sensor_type: wp.array(dtype=int),
  sensor_datatype: wp.array(dtype=int),
  sensor_objtype: wp.array(dtype=int),
  sensor_objid: wp.array(dtype=int),
  sensor_intprm: wp.array2d(dtype=int),
  sensor_dim: wp.array(dtype=int),
  sensor_adr: wp.array(dtype=int),
  sensor_cutoff: wp.array(dtype=float),
  sensor_acc_adr: wp.array(dtype=int),
  sensor_contact_adr: wp.array(dtype=int),
  # Data in:
  njmax_in: int,
  ncon_in: wp.array(dtype=int),
  xpos_in: wp.array2d(dtype=wp.vec3),
  xipos_in: wp.array2d(dtype=wp.vec3),
  geom_xpos_in: wp.array2d(dtype=wp.vec3),
  site_xpos_in: wp.array2d(dtype=wp.vec3),
  site_xmat_in: wp.array2d(dtype=wp.mat33),
  cam_xpos_in: wp.array2d(dtype=wp.vec3),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  cvel_in: wp.array2d(dtype=wp.spatial_vector),
  actuator_force_in: wp.array2d(dtype=float),
  qfrc_actuator_in: wp.array2d(dtype=float),
  contact_dist_in: wp.array(dtype=float),
  contact_pos_in: wp.array(dtype=wp.vec3),
  contact_frame_in: wp.array(dtype=wp.mat33),
  contact_friction_in: wp.array(dtype=vec5),
  contact_dim_in: wp.array(dtype=int),
  contact_efc_address_in: wp.array2d(dtype=int),
  efc_force_in: wp.array2d(dtype=float),
  cacc_in: wp.array2d(dtype=wp.spatial_vector),
  cfrc_int_in: wp.array2d(dtype=wp.spatial_vector),
  sensor_contact_nmatch_in: wp.array2d(dtype=int),
  sensor_contact_matchid_in: wp.array3d(dtype=int),
  sensor_contact_direction_in: wp.array3d(dtype=float),
  # Data out:
  sensordata_out: wp.array2d(dtype=float),
):
  worldid, accid = wp.tid()
  sensorid = sensor_acc_adr[accid]
  sensortype = sensor_type[sensorid]
  objid = sensor_objid[sensorid]
  out = sensordata_out[worldid]

  if sensortype == int(SensorType.CONTACT.value):
    dataspec = sensor_intprm[sensorid, 0]
    dim = sensor_dim[sensorid]
    objtype = sensor_objtype[sensorid]

    # found, force, torque, dist, pos, normal, tangent
    # TODO(thowell): precompute slot size
    found = False
    force = False
    torque = False
    dist = False
    pos = False
    normal = False
    tangent = False

    size = int(0)
    for i in range(7):
      if dataspec & (1 << i):
        if i == 0:
          found = True
          size += 1
        elif i == 1:
          force = True
          size += 3
        elif i == 2:
          torque = True
          size += 3
        elif i == 3:
          dist = True
          size += 1
        elif i == 4:
          pos = True
          size += 3
        elif i == 5:
          normal = True
          size += 3
        elif i == 6:
          tangent = True
          size += 3

    num = dim // size  # number of slots

    adr = sensor_adr[sensorid]

    # TODO(team): precompute sensorid to contactsensorid mapping
    contactsensorid = int(0)
    for i in range(sensor_contact_adr.size):
      if sensorid == sensor_contact_adr[i]:
        contactsensorid = i
        break

    nmatch = sensor_contact_nmatch_in[worldid, contactsensorid]

    for i in range(wp.min(nmatch, num)):
      # sorted contact id
      cid = sensor_contact_matchid_in[worldid, contactsensorid, i]

      # contact direction
      dir = sensor_contact_direction_in[worldid, contactsensorid, i]

      adr_slot = adr + i * size

      if found:
        out[adr_slot] = float(nmatch)
        adr_slot += 1
      if force or torque:
        contact_forcetorque = support.contact_force_fn(
          opt_cone,
          njmax_in,
          ncon_in,
          contact_frame_in,
          contact_friction_in,
          contact_dim_in,
          contact_efc_address_in,
          efc_force_in,
          worldid,
          cid,
          False,
        )
      if force:
        out[adr_slot + 0] = contact_forcetorque[0]
        out[adr_slot + 1] = contact_forcetorque[1]
        out[adr_slot + 2] = dir * contact_forcetorque[2]
        adr_slot += 3
      if torque:
        out[adr_slot + 0] = contact_forcetorque[3]
        out[adr_slot + 1] = contact_forcetorque[4]
        out[adr_slot + 2] = dir * contact_forcetorque[5]
        adr_slot += 3
      if dist:
        out[adr_slot] = contact_dist_in[cid]
        adr_slot += 1
      if pos:
        contact_pos = contact_pos_in[cid]
        out[adr_slot + 0] = contact_pos[0]
        out[adr_slot + 1] = contact_pos[1]
        out[adr_slot + 2] = contact_pos[2]
        adr_slot += 3
      if normal:
        contact_normal = contact_frame_in[cid][0]
        out[adr_slot + 0] = dir * contact_normal[0]
        out[adr_slot + 1] = dir * contact_normal[1]
        out[adr_slot + 2] = dir * contact_normal[2]
        adr_slot += 3
      if tangent:
        contact_tangent = contact_frame_in[cid][1]
        out[adr_slot + 0] = dir * contact_tangent[0]
        out[adr_slot + 1] = dir * contact_tangent[1]
        out[adr_slot + 2] = dir * contact_tangent[2]
        adr_slot += 3

    # zero remaining slots
    for i in range(nmatch, num):
      for j in range(size):
        out[adr + i * size + j] = 0.0

  elif sensortype == int(SensorType.ACCELEROMETER.value):
    vec3 = _accelerometer(
      body_rootid, site_bodyid, site_xpos_in, site_xmat_in, subtree_com_in, cvel_in, cacc_in, worldid, objid
    )
    _write_vector(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, vec3, out)
  elif sensortype == int(SensorType.FORCE.value):
    vec3 = _force(site_bodyid, site_xmat_in, cfrc_int_in, worldid, objid)
    _write_vector(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, vec3, out)
  elif sensortype == int(SensorType.TORQUE.value):
    vec3 = _torque(body_rootid, site_bodyid, site_xpos_in, site_xmat_in, subtree_com_in, cfrc_int_in, worldid, objid)
    _write_vector(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, vec3, out)
  elif sensortype == int(SensorType.ACTUATORFRC.value):
    val = _actuator_force(actuator_force_in, worldid, objid)
    _write_scalar(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, val, out)
  elif sensortype == int(SensorType.JOINTACTFRC.value):
    val = _joint_actuator_force(jnt_dofadr, qfrc_actuator_in, worldid, objid)
    _write_scalar(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, val, out)
  elif sensortype == int(SensorType.FRAMELINACC.value):
    objtype = sensor_objtype[sensorid]
    vec3 = _framelinacc(
      body_rootid,
      geom_bodyid,
      site_bodyid,
      cam_bodyid,
      xpos_in,
      xipos_in,
      geom_xpos_in,
      site_xpos_in,
      cam_xpos_in,
      subtree_com_in,
      cvel_in,
      cacc_in,
      worldid,
      objid,
      objtype,
    )
    _write_vector(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, vec3, out)
  elif sensortype == int(SensorType.FRAMEANGACC.value):
    objtype = sensor_objtype[sensorid]
    vec3 = _frameangacc(
      geom_bodyid,
      site_bodyid,
      cam_bodyid,
      cacc_in,
      worldid,
      objid,
      objtype,
    )
    _write_vector(sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, vec3, out)


@wp.kernel
def _sensor_touch_zero(
  # Model:
  sensor_adr: wp.array(dtype=int),
  sensor_touch_adr: wp.array(dtype=int),
  # Data out:
  sensordata_out: wp.array2d(dtype=float),
):
  worldid, sensortouchadrid = wp.tid()
  sensorid = sensor_touch_adr[sensortouchadrid]
  adr = sensor_adr[sensorid]
  sensordata_out[worldid, adr] = 0.0


@wp.kernel
def _sensor_touch(
  # Model:
  opt_cone: int,
  geom_bodyid: wp.array(dtype=int),
  site_type: wp.array(dtype=int),
  site_bodyid: wp.array(dtype=int),
  site_size: wp.array(dtype=wp.vec3),
  sensor_objid: wp.array(dtype=int),
  sensor_adr: wp.array(dtype=int),
  sensor_touch_adr: wp.array(dtype=int),
  # Data in:
  ncon_in: wp.array(dtype=int),
  site_xpos_in: wp.array2d(dtype=wp.vec3),
  site_xmat_in: wp.array2d(dtype=wp.mat33),
  contact_pos_in: wp.array(dtype=wp.vec3),
  contact_frame_in: wp.array(dtype=wp.mat33),
  contact_dim_in: wp.array(dtype=int),
  contact_geom_in: wp.array(dtype=wp.vec2i),
  contact_efc_address_in: wp.array2d(dtype=int),
  contact_worldid_in: wp.array(dtype=int),
  efc_force_in: wp.array2d(dtype=float),
  # Data out:
  sensordata_out: wp.array2d(dtype=float),
):
  conid, sensortouchadrid = wp.tid()

  if conid > ncon_in[0]:
    return

  sensorid = sensor_touch_adr[sensortouchadrid]

  objid = sensor_objid[sensorid]
  bodyid = site_bodyid[objid]

  # find contact in sensor zone, add normal force

  # contacting bodies
  geom = contact_geom_in[conid]
  conbody = wp.vec2i(geom_bodyid[geom[0]], geom_bodyid[geom[1]])

  # select contacts involving sensorized body
  worldid = contact_worldid_in[conid]
  efc_address0 = contact_efc_address_in[conid, 0]
  if efc_address0 >= 0 and (bodyid == conbody[0] or bodyid == conbody[1]):
    # get contact normal force
    normalforce = efc_force_in[worldid, efc_address0]

    if opt_cone == int(ConeType.PYRAMIDAL.value):
      dim = contact_dim_in[conid]
      for i in range(1, 2 * (dim - 1)):
        normalforce += efc_force_in[worldid, contact_efc_address_in[conid, i]]

    if normalforce <= 0.0:
      return

    # convert contact normal force to global frame, normalize
    frame = contact_frame_in[conid]
    conray = wp.vec3(frame[0, 0], frame[0, 1], frame[0, 2]) * normalforce
    conray, _ = math.normalize_with_norm(conray)

    # flip ray direction if sensor is on body2
    if bodyid == conbody[1]:
      conray = -conray

    # add if ray-zone intersection (always true when contact.pos inside zone)
    if (
      ray.ray_geom(
        site_xpos_in[worldid, objid],
        site_xmat_in[worldid, objid],
        site_size[objid],
        contact_pos_in[conid],
        conray,
        site_type[objid],
      )
      >= 0.0
    ):
      adr = sensor_adr[sensorid]
      wp.atomic_add(sensordata_out[worldid], adr, normalforce)


@wp.kernel
def _sensor_tactile_zero(
  # Model:
  sensor_type: wp.array(dtype=int),
  sensor_dim: wp.array(dtype=int),
  sensor_adr: wp.array(dtype=int),
  # Data out:
  sensordata_out: wp.array2d(dtype=float),
):
  worldid, sensorid = wp.tid()

  if sensor_type[sensorid] != int(SensorType.TACTILE.value):
    return

  for i in range(sensor_dim[sensorid]):
    sensordata_out[worldid, sensor_adr[sensorid] + i] = 0.0


@wp.func
def _transform_spatial(vec: wp.spatial_vector, dif: wp.vec3) -> wp.vec3:
  return wp.spatial_bottom(vec) - wp.cross(dif, wp.spatial_top(vec))


@wp.kernel
def _sensor_tactile(
  # Model:
  body_rootid: wp.array(dtype=int),
  body_weldid: wp.array(dtype=int),
  geom_bodyid: wp.array(dtype=int),
  mesh_vertadr: wp.array(dtype=int),
  mesh_vert: wp.array(dtype=wp.vec3),
  mesh_normaladr: wp.array(dtype=int),
  mesh_normal: wp.array(dtype=wp.vec3),
  mesh_quat: wp.array(dtype=wp.quat),
  sensor_objid: wp.array(dtype=int),
  sensor_refid: wp.array(dtype=int),
  sensor_dim: wp.array(dtype=int),
  sensor_adr: wp.array(dtype=int),
  plugin: wp.array(dtype=int),
  plugin_attr: wp.array(dtype=wp.vec3f),
  geom_plugin_index: wp.array(dtype=int),
  taxel_vertadr: wp.array(dtype=int),
  taxel_sensorid: wp.array(dtype=int),
  # Data in:
  ncon_in: wp.array(dtype=int),
  geom_xpos_in: wp.array2d(dtype=wp.vec3),
  geom_xmat_in: wp.array2d(dtype=wp.mat33),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  cvel_in: wp.array2d(dtype=wp.spatial_vector),
  contact_geom_in: wp.array(dtype=wp.vec2i),
  contact_worldid_in: wp.array(dtype=int),
  # Data out:
  sensordata_out: wp.array2d(dtype=float),
):
  conid, taxelid = wp.tid()

  if conid >= ncon_in[0]:
    return

  worldid = contact_worldid_in[conid]

  # get sensor_id
  sensor_id = taxel_sensorid[taxelid]

  # get parent weld id
  mesh_id = sensor_objid[sensor_id]
  geom_id = sensor_refid[sensor_id]
  parent_body = geom_bodyid[geom_id]
  parent_weld = body_weldid[parent_body]

  # contact geom
  body1 = body_weldid[geom_bodyid[contact_geom_in[conid][0]]]
  body2 = body_weldid[geom_bodyid[contact_geom_in[conid][1]]]
  if body1 == parent_weld:
    geom = contact_geom_in[conid][1]
  elif body2 == parent_weld:
    geom = contact_geom_in[conid][0]
  else:
    return
  body = geom_bodyid[geom]

  # vertex local position
  vertid = taxel_vertadr[taxelid] - mesh_vertadr[mesh_id]
  pos = mesh_vert[vertid + mesh_vertadr[mesh_id]]

  # position in global frame
  xpos = geom_xmat_in[worldid, geom_id] @ pos
  xpos += geom_xpos_in[worldid, geom_id]

  # position in other geom frame
  tmp = xpos - geom_xpos_in[worldid, geom]
  lpos = wp.transpose(geom_xmat_in[worldid, geom]) @ tmp

  # compute distance
  plugin_id = geom_plugin_index[geom]
  depth = wp.min(sdf(int(GeomType.SDF.value), lpos, plugin_attr[plugin_id], plugin[plugin_id]), 0.0)
  if depth >= 0.0:
    return

  # get velocity in global
  vel_sensor = _transform_spatial(cvel_in[worldid, parent_weld], xpos - subtree_com_in[worldid, body_rootid[parent_weld]])
  vel_other = _transform_spatial(
    cvel_in[worldid, body], geom_xpos_in[worldid, geom] - subtree_com_in[worldid, body_rootid[body]]
  )
  vel_rel = vel_sensor - vel_other

  # get contact force/torque, rotate into node frame
  offset = mesh_normaladr[mesh_id] + 3 * vertid
  normal = math.rot_vec_quat(mesh_normal[offset], mesh_quat[mesh_id])
  tang1 = math.rot_vec_quat(mesh_normal[offset + 1], mesh_quat[mesh_id])
  tang2 = math.rot_vec_quat(mesh_normal[offset + 2], mesh_quat[mesh_id])
  kMaxDepth = 0.05
  pressure = depth / wp.max(kMaxDepth - depth, MJ_MINVAL)
  force = wp.mul(normal, pressure)

  # one row of mat^T * force
  forceT = wp.vec3()
  forceT[0] = wp.dot(force, normal)
  forceT[1] = wp.abs(wp.dot(vel_rel, tang1))
  forceT[2] = wp.abs(wp.dot(vel_rel, tang2))

  # add to sensor output
  dim = sensor_dim[sensor_id] / 3
  wp.atomic_add(sensordata_out[worldid], sensor_adr[sensor_id] + 0 * dim + vertid, forceT[0])
  wp.atomic_add(sensordata_out[worldid], sensor_adr[sensor_id] + 1 * dim + vertid, forceT[1])
  wp.atomic_add(sensordata_out[worldid], sensor_adr[sensor_id] + 2 * dim + vertid, forceT[2])


@wp.kernel
def _contact_match(
  # Model:
  sensor_objid: wp.array(dtype=int),
  sensor_refid: wp.array(dtype=int),
  sensor_contact_adr: wp.array(dtype=int),
  # Data in:
  ncon_in: wp.array(dtype=int),
  contact_dist_in: wp.array(dtype=float),
  contact_geom_in: wp.array(dtype=wp.vec2i),
  contact_worldid_in: wp.array(dtype=int),
  # Data out:
  sensor_contact_nmatch_out: wp.array2d(dtype=int),
  sensor_contact_matchid_out: wp.array3d(dtype=int),
  sensor_contact_criteria_out: wp.array3d(dtype=float),
  sensor_contact_direction_out: wp.array3d(dtype=float),
):
  contactsensorid, contactid = wp.tid()
  sensorid = sensor_contact_adr[contactsensorid]

  if contactid >= ncon_in[0]:
    return

  # sensor information
  objid = sensor_objid[sensorid]
  refid = sensor_refid[sensorid]

  # contact information
  geom = contact_geom_in[contactid]

  # geom-geom match
  geom0geom1 = objid == geom[0] and refid == geom[1]
  geom1geom0 = objid == geom[1] and refid == geom[0]
  if geom0geom1 or geom1geom0:
    worldid = contact_worldid_in[contactid]

    contactmatchid = wp.atomic_add(sensor_contact_nmatch_out[worldid], contactsensorid, 1)
    sensor_contact_matchid_out[worldid, contactsensorid, contactmatchid] = contactid

    # TODO(thowell): alternative criteria
    sensor_contact_criteria_out[worldid, contactsensorid, contactmatchid] = contact_dist_in[contactid]

    # contact direction
    if geom1geom0:
      sensor_contact_direction_out[worldid, contactsensorid, contactmatchid] = -1.0
    else:
      sensor_contact_direction_out[worldid, contactsensorid, contactmatchid] = 1.0

    return

  # TODO(thowell): alternative matching


@wp.kernel
def _contact_sort(
  # Data in:
  sensor_contact_nmatch_in: wp.array2d(dtype=int),
  sensor_contact_matchid_in: wp.array3d(dtype=int),
  sensor_contact_criteria_in: wp.array3d(dtype=float),
  # Data out:
  sensor_contact_matchid_out: wp.array3d(dtype=int),
):
  worldid, contactsensorid = wp.tid()

  nmatch = sensor_contact_nmatch_in[worldid, contactsensorid]

  # skip sort
  if nmatch <= 1:
    return

  criteria_tile = wp.tile_load(sensor_contact_criteria_in[worldid, contactsensorid], shape=MJ_MAXCONPAIR)
  matchid_tile = wp.tile_load(sensor_contact_matchid_in[worldid, contactsensorid], shape=MJ_MAXCONPAIR)
  wp.tile_sort(criteria_tile, matchid_tile)
  wp.tile_store(sensor_contact_matchid_out[worldid, contactsensorid], matchid_tile)


@event_scope
def sensor_acc(m: Model, d: Data):
  """Compute acceleration-dependent sensor values."""
  if m.opt.disableflags & DisableBit.SENSOR:
    return

  wp.launch(
    _sensor_touch_zero,
    dim=(d.nworld, m.sensor_touch_adr.size),
    inputs=[
      m.sensor_adr,
      m.sensor_touch_adr,
    ],
    outputs=[
      d.sensordata,
    ],
  )

  wp.launch(
    _sensor_touch,
    dim=(d.nconmax, m.sensor_touch_adr.size),
    inputs=[
      m.opt.cone,
      m.geom_bodyid,
      m.site_type,
      m.site_bodyid,
      m.site_size,
      m.sensor_objid,
      m.sensor_adr,
      m.sensor_touch_adr,
      d.ncon,
      d.site_xpos,
      d.site_xmat,
      d.contact.pos,
      d.contact.frame,
      d.contact.dim,
      d.contact.geom,
      d.contact.efc_address,
      d.contact.worldid,
      d.efc.force,
    ],
    outputs=[
      d.sensordata,
    ],
  )

  wp.launch(
    _sensor_tactile_zero,
    dim=(d.nworld, m.nsensordata),
    inputs=[
      m.sensor_type,
      m.sensor_dim,
      m.sensor_adr,
    ],
    outputs=[
      d.sensordata,
    ],
  )

  wp.launch(
    _sensor_tactile,
    dim=(d.nconmax, m.nsensortaxel),
    inputs=[
      m.body_rootid,
      m.body_weldid,
      m.geom_bodyid,
      m.mesh_vertadr,
      m.mesh_vert,
      m.mesh_normaladr,
      m.mesh_normal,
      m.mesh_quat,
      m.sensor_objid,
      m.sensor_refid,
      m.sensor_dim,
      m.sensor_adr,
      m.plugin,
      m.plugin_attr,
      m.geom_plugin_index,
      m.taxel_vertadr,
      m.taxel_sensorid,
      d.ncon,
      d.geom_xpos,
      d.geom_xmat,
      d.subtree_com,
      d.cvel,
      d.contact.geom,
      d.contact.worldid,
    ],
    outputs=[
      d.sensordata,
    ],
  )

  if m.sensor_contact_adr.size:
    # match criteria
    d.sensor_contact_nmatch.zero_()
    d.sensor_contact_matchid.zero_()
    d.sensor_contact_criteria.zero_()

    wp.launch(
      _contact_match,
      dim=(m.sensor_contact_adr.size, d.nconmax),
      inputs=[
        m.sensor_objid,
        m.sensor_refid,
        m.sensor_contact_adr,
        d.ncon,
        d.contact.dist,
        d.contact.geom,
        d.contact.worldid,
      ],
      outputs=[
        d.sensor_contact_nmatch,
        d.sensor_contact_matchid,
        d.sensor_contact_criteria,
        d.sensor_contact_direction,
      ],
    )

    # sorting
    wp.launch_tiled(
      _contact_sort,
      dim=(d.nworld, m.sensor_contact_adr.size),
      inputs=[
        d.sensor_contact_nmatch,
        d.sensor_contact_matchid,
        d.sensor_contact_criteria,
      ],
      outputs=[
        d.sensor_contact_matchid,
      ],
      block_dim=m.block_dim.contact_sort,
    )

  if m.sensor_rne_postconstraint:
    smooth.rne_postconstraint(m, d)

  wp.launch(
    _sensor_acc,
    dim=(d.nworld, m.sensor_acc_adr.size),
    inputs=[
      m.opt.cone,
      m.body_rootid,
      m.jnt_dofadr,
      m.geom_bodyid,
      m.site_bodyid,
      m.cam_bodyid,
      m.sensor_type,
      m.sensor_datatype,
      m.sensor_objtype,
      m.sensor_objid,
      m.sensor_intprm,
      m.sensor_dim,
      m.sensor_adr,
      m.sensor_cutoff,
      m.sensor_acc_adr,
      m.sensor_contact_adr,
      d.njmax,
      d.ncon,
      d.xpos,
      d.xipos,
      d.geom_xpos,
      d.site_xpos,
      d.site_xmat,
      d.cam_xpos,
      d.subtree_com,
      d.cvel,
      d.actuator_force,
      d.qfrc_actuator,
      d.contact.dist,
      d.contact.pos,
      d.contact.frame,
      d.contact.friction,
      d.contact.dim,
      d.contact.efc_address,
      d.efc.force,
      d.cacc,
      d.cfrc_int,
      d.sensor_contact_nmatch,
      d.sensor_contact_matchid,
      d.sensor_contact_direction,
    ],
    outputs=[d.sensordata],
  )

  wp.launch(
    _tendon_actuator_force_zero,
    dim=(d.nworld, m.sensor_tendonactfrc_adr.size),
    inputs=[
      m.sensor_adr,
      m.sensor_tendonactfrc_adr,
    ],
    outputs=[
      d.sensordata,
    ],
  )

  wp.launch(
    _tendon_actuator_force,
    dim=(d.nworld, m.sensor_tendonactfrc_adr.size, m.nu),
    inputs=[
      m.actuator_trntype,
      m.actuator_trnid,
      m.sensor_objid,
      m.sensor_adr,
      m.sensor_tendonactfrc_adr,
      d.actuator_force,
    ],
    outputs=[
      d.sensordata,
    ],
  )

  wp.launch(
    _tendon_actuator_force_cutoff,
    dim=(d.nworld, m.sensor_tendonactfrc_adr.size),
    inputs=[
      m.sensor_datatype,
      m.sensor_adr,
      m.sensor_cutoff,
      m.sensor_tendonactfrc_adr,
      d.sensordata,
    ],
    outputs=[d.sensordata],
  )

  wp.launch(
    _limit_frc_zero,
    dim=(d.nworld, m.sensor_limitfrc_adr.size),
    inputs=[m.sensor_adr, m.sensor_limitfrc_adr],
    outputs=[d.sensordata],
  )

  wp.launch(
    _limit_frc,
    dim=(d.nworld, d.njmax, m.sensor_limitfrc_adr.size),
    inputs=[
      m.sensor_datatype,
      m.sensor_objid,
      m.sensor_adr,
      m.sensor_cutoff,
      m.sensor_limitfrc_adr,
      d.ne,
      d.nf,
      d.nl,
      d.efc.type,
      d.efc.id,
      d.efc.force,
    ],
    outputs=[
      d.sensordata,
    ],
  )


@wp.kernel
def _energy_pos_zero(
  # Data out:
  energy_out: wp.array(dtype=wp.vec2),
):
  worldid = wp.tid()
  energy_out[worldid][0] = 0.0


@wp.kernel
def _energy_pos_gravity(
  # Model:
  opt_gravity: wp.array(dtype=wp.vec3),
  body_mass: wp.array2d(dtype=float),
  # Data in:
  xipos_in: wp.array2d(dtype=wp.vec3),
  # Data out:
  energy_out: wp.array(dtype=wp.vec2),
):
  worldid, bodyid = wp.tid()
  gravity = opt_gravity[worldid]
  bodyid += 1  # skip world body

  energy = wp.vec2(
    body_mass[worldid, bodyid] * wp.dot(gravity, xipos_in[worldid, bodyid]),
    0.0,
  )

  wp.atomic_sub(energy_out, worldid, energy)


@wp.kernel
def _energy_pos_passive_joint(
  # Model:
  qpos_spring: wp.array2d(dtype=float),
  jnt_type: wp.array(dtype=int),
  jnt_qposadr: wp.array(dtype=int),
  jnt_stiffness: wp.array2d(dtype=float),
  # Data in:
  qpos_in: wp.array2d(dtype=float),
  # Data out:
  energy_out: wp.array(dtype=wp.vec2),
):
  worldid, jntid = wp.tid()
  stiffness = jnt_stiffness[worldid, jntid]

  if stiffness == 0.0:
    return

  padr = jnt_qposadr[jntid]
  jnttype = jnt_type[jntid]

  if jnttype == int(JointType.FREE.value):
    dif0 = wp.vec3(
      qpos_in[worldid, padr + 0] - qpos_spring[worldid, padr + 0],
      qpos_in[worldid, padr + 1] - qpos_spring[worldid, padr + 1],
      qpos_in[worldid, padr + 2] - qpos_spring[worldid, padr + 2],
    )

    # convert quaternion difference into angular "velocity"
    quat1 = wp.quat(
      qpos_in[worldid, padr + 3],
      qpos_in[worldid, padr + 4],
      qpos_in[worldid, padr + 5],
      qpos_in[worldid, padr + 6],
    )
    quat1 = wp.normalize(quat1)

    quat_spring = wp.quat(
      qpos_spring[worldid, padr + 3],
      qpos_spring[worldid, padr + 4],
      qpos_spring[worldid, padr + 5],
      qpos_spring[worldid, padr + 6],
    )

    dif1 = math.quat_sub(quat1, quat_spring)

    energy = wp.vec2(
      0.5 * stiffness * (wp.dot(dif0, dif0) + wp.dot(dif1, dif1)),
      0.0,
    )

    wp.atomic_add(energy_out, worldid, energy)

  elif jnttype == int(JointType.BALL.value):
    quat = wp.quat(
      qpos_in[worldid, padr + 0],
      qpos_in[worldid, padr + 1],
      qpos_in[worldid, padr + 2],
      qpos_in[worldid, padr + 3],
    )
    quat = wp.normalize(quat)

    quat_spring = wp.quat(
      qpos_spring[worldid, padr + 0],
      qpos_spring[worldid, padr + 1],
      qpos_spring[worldid, padr + 2],
      qpos_spring[worldid, padr + 3],
    )

    dif = math.quat_sub(quat, quat_spring)
    energy = wp.vec2(
      0.5 * stiffness * wp.dot(dif, dif),
      0.0,
    )
    wp.atomic_add(energy_out, worldid, energy)
  elif jnttype == int(JointType.SLIDE.value) or jnttype == int(JointType.HINGE.value):
    dif_ = qpos_in[worldid, padr] - qpos_spring[worldid, padr]
    energy = wp.vec2(
      0.5 * stiffness * dif_ * dif_,
      0.0,
    )
    wp.atomic_add(energy_out, worldid, energy)


@wp.kernel
def _energy_pos_passive_tendon(
  # Model:
  tendon_stiffness: wp.array2d(dtype=float),
  tendon_lengthspring: wp.array2d(dtype=wp.vec2),
  # Data in:
  ten_length_in: wp.array2d(dtype=float),
  # Data out:
  energy_out: wp.array(dtype=wp.vec2),
):
  worldid, tenid = wp.tid()

  stiffness = tendon_stiffness[worldid, tenid]

  if stiffness == 0.0:
    return

  length = ten_length_in[worldid, tenid]

  # compute spring displacement
  lengthspring = tendon_lengthspring[worldid, tenid]
  lower = lengthspring[0]
  upper = lengthspring[1]

  if length > upper:
    displacement = upper - length
  elif length < lower:
    displacement = lower - length
  else:
    displacement = 0.0

  energy = wp.vec2(0.5 * stiffness * displacement * displacement, 0.0)
  wp.atomic_add(energy_out, worldid, energy)


def energy_pos(m: Model, d: Data):
  """Position-dependent energy (potential)."""
  wp.launch(_energy_pos_zero, dim=(d.nworld,), outputs=[d.energy])

  # init potential energy: -sum_i(body_i.mass * dot(gravity, body_i.pos))
  if not m.opt.disableflags & DisableBit.GRAVITY:
    wp.launch(
      _energy_pos_gravity, dim=(d.nworld, m.nbody - 1), inputs=[m.opt.gravity, m.body_mass, d.xipos], outputs=[d.energy]
    )

  if not m.opt.disableflags & DisableBit.PASSIVE:
    # add joint-level springs
    wp.launch(
      _energy_pos_passive_joint,
      dim=(d.nworld, m.njnt),
      inputs=[
        m.qpos_spring,
        m.jnt_type,
        m.jnt_qposadr,
        m.jnt_stiffness,
        d.qpos,
      ],
      outputs=[d.energy],
    )

    # add tendon-level springs
    if m.ntendon:
      wp.launch(
        _energy_pos_passive_tendon,
        dim=(d.nworld, m.ntendon),
        inputs=[
          m.tendon_stiffness,
          m.tendon_lengthspring,
          d.ten_length,
        ],
        outputs=[d.energy],
      )

    # TODO(team): flex


@cache_kernel
def _energy_vel_kinetic(nv: int):
  @nested_kernel
  def energy_vel_kinetic(
    # Data in:
    qvel_in: wp.array2d(dtype=float),
    # In:
    Mqvel: wp.array2d(dtype=float),
    # Out:
    energy_out: wp.array(dtype=wp.vec2),
  ):
    worldid = wp.tid()

    qvel_tile = wp.tile_load(qvel_in[worldid], shape=wp.static(nv))
    Mqvel_tile = wp.tile_load(Mqvel[worldid], shape=wp.static(nv))

    # qvel * (M @ qvel)
    qvelMqvel_tile = wp.tile_map(wp.mul, qvel_tile, Mqvel_tile)

    # sum(qvel * (M @ qvel))
    quadratic_tile = wp.tile_reduce(wp.add, qvelMqvel_tile)

    energy_out[worldid][1] = 0.5 * quadratic_tile[0]

  return energy_vel_kinetic


def energy_vel(m: Model, d: Data):
  """Velocity-dependent energy (kinetic)."""

  # kinetic energy: 0.5 * qvel.T @ M @ qvel

  # M @ qvel
  skip = wp.zeros(d.nworld, dtype=bool)
  support.mul_m(m, d, d.efc.mv, d.qvel, skip)

  wp.launch_tiled(
    _energy_vel_kinetic(m.nv),
    dim=(d.nworld,),
    inputs=[d.qvel, d.efc.mv],
    outputs=[d.energy],
    block_dim=m.block_dim.energy_vel_kinetic,
  )
