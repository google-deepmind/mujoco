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
from mujoco.mjx.third_party.mujoco_warp._src.collision_sdf import get_sdf_params
from mujoco.mjx.third_party.mujoco_warp._src.collision_sdf import sdf
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MINVAL
from mujoco.mjx.third_party.mujoco_warp._src.types import ConeType
from mujoco.mjx.third_party.mujoco_warp._src.types import ConstraintType
from mujoco.mjx.third_party.mujoco_warp._src.types import ContactType
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import DataType
from mujoco.mjx.third_party.mujoco_warp._src.types import DisableBit
from mujoco.mjx.third_party.mujoco_warp._src.types import JointType
from mujoco.mjx.third_party.mujoco_warp._src.types import Model
from mujoco.mjx.third_party.mujoco_warp._src.types import ObjType
from mujoco.mjx.third_party.mujoco_warp._src.types import SensorType
from mujoco.mjx.third_party.mujoco_warp._src.types import TrnType
from mujoco.mjx.third_party.mujoco_warp._src.types import vec5
from mujoco.mjx.third_party.mujoco_warp._src.types import vec6
from mujoco.mjx.third_party.mujoco_warp._src.types import vec8
from mujoco.mjx.third_party.mujoco_warp._src.types import vec8i
from mujoco.mjx.third_party.mujoco_warp._src.util_misc import inside_geom
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import cache_kernel
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import nested_kernel

wp.set_module_options({"enable_backward": False})


@wp.func
def _write_scalar(
  # Model:
  sensor_type: wp.array(dtype=int),
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

  if cutoff > 0.0 and not (sensor_type[sensorid] == int(SensorType.GEOMFROMTO.value)):
    datatype = sensor_datatype[sensorid]
    if datatype == DataType.REAL:
      out[adr] = wp.clamp(sensor, -cutoff, cutoff)
      return
    elif datatype == DataType.POSITIVE:
      out[adr] = wp.min(sensor, cutoff)
      return

  out[adr] = sensor


@wp.func
def _write_vector(
  # Model:
  sensor_type: wp.array(dtype=int),
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

  if cutoff > 0.0 and not (sensor_type[sensorid] == int(SensorType.GEOMFROMTO.value)):
    datatype = sensor_datatype[sensorid]
    if datatype == DataType.REAL:
      for i in range(sensordim):
        out[adr + i] = wp.clamp(sensor[i], -cutoff, cutoff)
      return
    elif datatype == DataType.POSITIVE:
      for i in range(sensordim):
        out[adr + i] = wp.min(sensor[i], cutoff)
      return

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
  magnetic = opt_magnetic[worldid % opt_magnetic.shape[0]]
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
  # Out:
  pnt_out: wp.array2d(dtype=wp.vec3),
  vec_out: wp.array2d(dtype=wp.vec3),
):
  worldid, rfid = wp.tid()
  sensorid = sensor_rangefinder_adr[rfid]
  objid = sensor_objid[sensorid]
  site_xpos = site_xpos_in[worldid, objid]
  site_xmat = site_xmat_in[worldid, objid]

  pnt_out[worldid, rfid] = site_xpos
  vec_out[worldid, rfid] = wp.vec3(site_xmat[0, 2], site_xmat[1, 2], site_xmat[2, 2])


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
def _limit_pos(
  # Model:
  sensor_type: wp.array(dtype=int),
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
    if efc_type == ConstraintType.LIMIT_JOINT or efc_type == ConstraintType.LIMIT_TENDON:
      val = efc_pos_in[worldid, efcid] - efc_margin_in[worldid, efcid]
      _write_scalar(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, val, sensordata_out[worldid])


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
  if objtype == ObjType.BODY:
    xpos = xipos_in[worldid, objid]
  elif objtype == ObjType.XBODY:
    xpos = xpos_in[worldid, objid]
  elif objtype == ObjType.GEOM:
    xpos = geom_xpos_in[worldid, objid]
  elif objtype == ObjType.SITE:
    xpos = site_xpos_in[worldid, objid]
  elif objtype == ObjType.CAMERA:
    xpos = cam_xpos_in[worldid, objid]
  else:  # UNKNOWN
    xpos = wp.vec3(0.0)

  if refid == -1:
    return xpos

  if reftype == ObjType.BODY:
    xpos_ref = xipos_in[worldid, refid]
    xmat_ref = ximat_in[worldid, refid]
  elif objtype == ObjType.XBODY:
    xpos_ref = xpos_in[worldid, refid]
    xmat_ref = xmat_in[worldid, refid]
  elif reftype == ObjType.GEOM:
    xpos_ref = geom_xpos_in[worldid, refid]
    xmat_ref = geom_xmat_in[worldid, refid]
  elif reftype == ObjType.SITE:
    xpos_ref = site_xpos_in[worldid, refid]
    xmat_ref = site_xmat_in[worldid, refid]
  elif reftype == ObjType.CAMERA:
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
  if objtype == ObjType.BODY:
    xmat = ximat_in[worldid, objid]
    axis = wp.vec3(xmat[0, frame_axis], xmat[1, frame_axis], xmat[2, frame_axis])
  elif objtype == ObjType.XBODY:
    xmat = xmat_in[worldid, objid]
    axis = wp.vec3(xmat[0, frame_axis], xmat[1, frame_axis], xmat[2, frame_axis])
  elif objtype == ObjType.GEOM:
    xmat = geom_xmat_in[worldid, objid]
    axis = wp.vec3(xmat[0, frame_axis], xmat[1, frame_axis], xmat[2, frame_axis])
  elif objtype == ObjType.SITE:
    xmat = site_xmat_in[worldid, objid]
    axis = wp.vec3(xmat[0, frame_axis], xmat[1, frame_axis], xmat[2, frame_axis])
  elif objtype == ObjType.CAMERA:
    xmat = cam_xmat_in[worldid, objid]
    axis = wp.vec3(xmat[0, frame_axis], xmat[1, frame_axis], xmat[2, frame_axis])
  else:  # UNKNOWN
    axis = wp.vec3(xmat[0, frame_axis], xmat[1, frame_axis], xmat[2, frame_axis])

  if refid == -1:
    return axis

  if reftype == ObjType.BODY:
    xmat_ref = ximat_in[worldid, refid]
  elif reftype == ObjType.XBODY:
    xmat_ref = xmat_in[worldid, refid]
  elif reftype == ObjType.GEOM:
    xmat_ref = geom_xmat_in[worldid, refid]
  elif reftype == ObjType.SITE:
    xmat_ref = site_xmat_in[worldid, refid]
  elif reftype == ObjType.CAMERA:
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
  body_iquat_id = worldid % body_iquat.shape[0]
  geom_quat_id = worldid % geom_quat.shape[0]
  site_quat_id = worldid % site_quat.shape[0]
  cam_quat_id = worldid % cam_quat.shape[0]
  if objtype == ObjType.BODY:
    quat = math.mul_quat(xquat_in[worldid, objid], body_iquat[body_iquat_id, objid])
  elif objtype == ObjType.XBODY:
    quat = xquat_in[worldid, objid]
  elif objtype == ObjType.GEOM:
    quat = math.mul_quat(xquat_in[worldid, geom_bodyid[objid]], geom_quat[geom_quat_id, objid])
  elif objtype == ObjType.SITE:
    quat = math.mul_quat(xquat_in[worldid, site_bodyid[objid]], site_quat[site_quat_id, objid])
  elif objtype == ObjType.CAMERA:
    quat = math.mul_quat(xquat_in[worldid, cam_bodyid[objid]], cam_quat[cam_quat_id, objid])
  else:  # UNKNOWN
    quat = wp.quat(1.0, 0.0, 0.0, 0.0)

  if refid == -1:
    return quat

  if reftype == ObjType.BODY:
    refquat = math.mul_quat(xquat_in[worldid, refid], body_iquat[body_iquat_id, refid])
  elif reftype == ObjType.XBODY:
    refquat = xquat_in[worldid, refid]
  elif reftype == ObjType.GEOM:
    refquat = math.mul_quat(xquat_in[worldid, geom_bodyid[refid]], geom_quat[geom_quat_id, refid])
  elif reftype == ObjType.SITE:
    refquat = math.mul_quat(xquat_in[worldid, site_bodyid[refid]], site_quat[site_quat_id, refid])
  elif reftype == ObjType.CAMERA:
    refquat = math.mul_quat(xquat_in[worldid, cam_bodyid[refid]], cam_quat[cam_quat_id, refid])
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
  ngeom: int,
  opt_magnetic: wp.array(dtype=wp.vec3),
  body_geomnum: wp.array(dtype=int),
  body_geomadr: wp.array(dtype=int),
  body_iquat: wp.array2d(dtype=wp.quat),
  jnt_qposadr: wp.array(dtype=int),
  geom_type: wp.array(dtype=int),
  geom_bodyid: wp.array(dtype=int),
  geom_quat: wp.array2d(dtype=wp.quat),
  site_type: wp.array(dtype=int),
  site_bodyid: wp.array(dtype=int),
  site_size: wp.array(dtype=wp.vec3),
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
  nxn_pairid: wp.array(dtype=wp.vec2i),
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
  ten_length_in: wp.array2d(dtype=float),
  actuator_length_in: wp.array2d(dtype=float),
  # In:
  rangefinder_dist_in: wp.array2d(dtype=float),
  sensor_collision_in: wp.array4d(dtype=float),
  # Data out:
  sensordata_out: wp.array2d(dtype=float),
):
  worldid, posid = wp.tid()
  sensorid = sensor_pos_adr[posid]
  sensortype = sensor_type[sensorid]
  objid = sensor_objid[sensorid]
  out = sensordata_out[worldid]

  if sensortype == SensorType.MAGNETOMETER:
    vec3 = _magnetometer(opt_magnetic, site_xmat_in, worldid, objid)
    _write_vector(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, vec3, out)
  elif sensortype == SensorType.CAMPROJECTION:
    refid = sensor_refid[sensorid]
    vec2 = _cam_projection(
      cam_fovy, cam_resolution, cam_sensorsize, cam_intrinsic, site_xpos_in, cam_xpos_in, cam_xmat_in, worldid, objid, refid
    )
    _write_vector(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 2, vec2, out)
  elif sensortype == SensorType.RANGEFINDER:
    val = rangefinder_dist_in[worldid, rangefinder_sensor_adr[sensorid]]
    _write_scalar(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, val, out)
  elif sensortype == SensorType.JOINTPOS:
    val = _joint_pos(jnt_qposadr, qpos_in, worldid, objid)
    _write_scalar(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, val, out)
  elif sensortype == SensorType.TENDONPOS:
    val = _tendon_pos(ten_length_in, worldid, objid)
    _write_scalar(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, val, out)
  elif sensortype == SensorType.ACTUATORPOS:
    val = _actuator_pos(actuator_length_in, worldid, objid)
    _write_scalar(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, val, out)
  elif sensortype == SensorType.BALLQUAT:
    quat = _ball_quat(jnt_qposadr, qpos_in, worldid, objid)
    _write_vector(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 4, quat, out)
  elif sensortype == SensorType.FRAMEPOS:
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
    _write_vector(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, vec3, out)
  elif sensortype == SensorType.FRAMEXAXIS or sensortype == SensorType.FRAMEYAXIS or sensortype == SensorType.FRAMEZAXIS:
    objtype = sensor_objtype[sensorid]
    refid = sensor_refid[sensorid]
    reftype = sensor_reftype[sensorid]
    if sensortype == SensorType.FRAMEXAXIS:
      axis = 0
    elif sensortype == SensorType.FRAMEYAXIS:
      axis = 1
    elif sensortype == SensorType.FRAMEZAXIS:
      axis = 2
    vec3 = _frame_axis(
      ximat_in, xmat_in, geom_xmat_in, site_xmat_in, cam_xmat_in, worldid, objid, objtype, refid, reftype, axis
    )
    _write_vector(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, vec3, out)
  elif sensortype == SensorType.FRAMEQUAT:
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
    _write_vector(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 4, quat, out)
  elif sensortype == SensorType.SUBTREECOM:
    vec3 = _subtree_com(subtree_com_in, worldid, objid)
    _write_vector(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, vec3, out)
  elif sensortype == SensorType.GEOMDIST or sensortype == SensorType.GEOMNORMAL or sensortype == SensorType.GEOMFROMTO:
    objtype = sensor_objtype[sensorid]
    objid = sensor_objid[sensorid]
    reftype = sensor_reftype[sensorid]
    refid = sensor_refid[sensorid]

    # initialize
    dist = float(sensor_cutoff[sensorid])
    pnts = vec6(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    flip = bool(False)

    # check for flip direction
    if objtype == int(ObjType.BODY.value):
      n1 = body_geomnum[objid]
      id1 = body_geomadr[objid]
    else:
      n1 = 1
      id1 = objid
    if reftype == int(ObjType.BODY.value):
      n2 = body_geomnum[refid]
      id2 = body_geomadr[refid]
    else:
      n2 = 1
      id2 = refid

    for geom1 in range(n1):
      geomid1 = id1 + geom1
      for geom2 in range(n2):
        geomid2 = id2 + geom2

        if geomid1 <= geomid2:
          pairid = math.upper_tri_index(ngeom, geomid1, geomid2)
        else:
          pairid = math.upper_tri_index(ngeom, geomid2, geomid1)
        collisionid = nxn_pairid[pairid][1]

        for i in range(8):
          dist_new = sensor_collision_in[worldid, collisionid, i, 0]

          if dist_new < dist:
            dist = dist_new

            if sensortype == SensorType.GEOMNORMAL or sensortype == SensorType.GEOMFROMTO:
              pnts = vec6(
                sensor_collision_in[worldid, collisionid, i, 1],
                sensor_collision_in[worldid, collisionid, i, 2],
                sensor_collision_in[worldid, collisionid, i, 3],
                sensor_collision_in[worldid, collisionid, i, 4],
                sensor_collision_in[worldid, collisionid, i, 5],
                sensor_collision_in[worldid, collisionid, i, 6],
              )

            if geom_type[geomid1] > geom_type[geomid2]:
              flip = True
            elif geom_type[geomid1] == geom_type[geomid2]:
              flip = geomid1 > geomid2
            else:
              flip = False
    if sensortype == int(SensorType.GEOMDIST.value):
      _write_scalar(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, dist, out)
    elif sensortype == int(SensorType.GEOMNORMAL.value):
      if dist <= sensor_cutoff[sensorid]:
        normal = wp.normalize(wp.vec3(pnts[3] - pnts[0], pnts[4] - pnts[1], pnts[5] - pnts[2]))
        if flip:
          normal *= -1.0
      else:
        normal = wp.vec3(0.0, 0.0, 0.0)
      _write_vector(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, normal, out)
    elif sensortype == int(SensorType.GEOMFROMTO.value):
      if dist <= sensor_cutoff[sensorid]:
        if flip:
          fromto = vec6(pnts[3], pnts[4], pnts[5], pnts[0], pnts[1], pnts[2])
        else:
          fromto = pnts
      else:
        fromto = vec6(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
      _write_vector(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 6, fromto, out)
  elif sensortype == SensorType.INSIDESITE:
    objtype = sensor_objtype[sensorid]
    if objtype == ObjType.XBODY:
      xpos = xpos_in[worldid, objid]
    elif objtype == ObjType.BODY:
      xpos = xipos_in[worldid, objid]
    elif objtype == ObjType.GEOM:
      xpos = geom_xpos_in[worldid, objid]
    elif objtype == ObjType.SITE:
      xpos = site_xpos_in[worldid, objid]
    elif objtype == ObjType.CAMERA:
      xpos = cam_xpos_in[worldid, objid]
    else:
      return  # should not occur
    refid = sensor_refid[sensorid]
    val_bool = inside_geom(site_xpos_in[worldid, refid], site_xmat_in[worldid, refid], site_size[refid], site_type[refid], xpos)
    _write_scalar(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, float(val_bool), out)
  elif sensortype == SensorType.E_POTENTIAL:
    val = energy_in[worldid][0]
    _write_scalar(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, val, out)
  elif sensortype == SensorType.E_KINETIC:
    val = energy_in[worldid][1]
    _write_scalar(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, val, out)
  elif sensortype == SensorType.CLOCK:
    val = _clock(time_in, worldid)
    _write_scalar(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, val, out)


@wp.kernel
def _sensor_collision(
  # Model:
  ngeom: int,
  nxn_pairid: wp.array(dtype=wp.vec2i),
  # Data in:
  contact_dist_in: wp.array(dtype=float),
  contact_pos_in: wp.array(dtype=wp.vec3),
  contact_frame_in: wp.array(dtype=wp.mat33),
  contact_geom_in: wp.array(dtype=wp.vec2i),
  contact_worldid_in: wp.array(dtype=int),
  contact_type_in: wp.array(dtype=int),
  contact_geomcollisionid_in: wp.array(dtype=int),
  nacon_in: wp.array(dtype=int),
  # Out:
  sensor_collision_out: wp.array4d(dtype=float),
):
  conid = wp.tid()

  if conid >= nacon_in[0]:
    return

  if not contact_type_in[conid] & ContactType.SENSOR:
    return

  geom = contact_geom_in[conid]
  if geom[0] <= geom[1]:
    pairid = math.upper_tri_index(ngeom, geom[0], geom[1])
  else:
    pairid = math.upper_tri_index(ngeom, geom[1], geom[0])

  worldid = contact_worldid_in[conid]
  collisionid = nxn_pairid[pairid][1]
  geomcollisionid = contact_geomcollisionid_in[conid]

  dist = contact_dist_in[conid]
  pos = contact_pos_in[conid]
  frame = contact_frame_in[conid]
  normal = wp.vec3(frame[0, 0], frame[0, 1], frame[0, 2])
  pnt1 = pos - 0.5 * dist * normal
  pnt2 = pos + 0.5 * dist * normal

  sensor_collision_out[worldid, collisionid, geomcollisionid, 0] = dist
  sensor_collision_out[worldid, collisionid, geomcollisionid, 1] = pnt1[0]
  sensor_collision_out[worldid, collisionid, geomcollisionid, 2] = pnt1[1]
  sensor_collision_out[worldid, collisionid, geomcollisionid, 3] = pnt1[2]
  sensor_collision_out[worldid, collisionid, geomcollisionid, 4] = pnt2[0]
  sensor_collision_out[worldid, collisionid, geomcollisionid, 5] = pnt2[1]
  sensor_collision_out[worldid, collisionid, geomcollisionid, 6] = pnt2[2]


@event_scope
def sensor_pos(m: Model, d: Data):
  """Compute position-dependent sensor values."""
  if m.opt.disableflags & DisableBit.SENSOR:
    return

  # rangefinder
  rangefinder_dist = wp.empty((d.nworld, m.nrangefinder), dtype=float)
  if m.sensor_rangefinder_adr.size > 0:
    rangefinder_pnt = wp.empty((d.nworld, m.nrangefinder), dtype=wp.vec3)
    rangefinder_vec = wp.empty((d.nworld, m.nrangefinder), dtype=wp.vec3)
    rangefinder_geomid = wp.empty((d.nworld, m.nrangefinder), dtype=int)

    # get position and direction
    wp.launch(
      _sensor_rangefinder_init,
      dim=(d.nworld, m.sensor_rangefinder_adr.size),
      inputs=[m.sensor_objid, m.sensor_rangefinder_adr, d.site_xpos, d.site_xmat],
      outputs=[rangefinder_pnt, rangefinder_vec],
    )

    # get distances
    ray.rays(
      m,
      d,
      rangefinder_pnt,
      rangefinder_vec,
      vec6(wp.inf, wp.inf, wp.inf, wp.inf, wp.inf, wp.inf),
      True,
      m.sensor_rangefinder_bodyid,
      rangefinder_dist,
      rangefinder_geomid,
    )

  if m.sensor_e_potential:
    energy_pos(m, d)

  if m.sensor_e_kinetic:
    energy_vel(m, d)

  # collision sensors (distance, normal, fromto)
  sensor_collision = wp.empty((d.nworld, m.nsensorcollision, 8, 7), dtype=float)
  sensor_collision.fill_(1.0e32)
  if m.nsensorcollision:
    wp.launch(
      _sensor_collision,
      dim=d.naconmax,
      inputs=[
        m.ngeom,
        m.nxn_pairid,
        d.contact.dist,
        d.contact.pos,
        d.contact.frame,
        d.contact.geom,
        d.contact.worldid,
        d.contact.type,
        d.contact.geomcollisionid,
        d.nacon,
      ],
      outputs=[sensor_collision],
    )

  wp.launch(
    _sensor_pos,
    dim=(d.nworld, m.sensor_pos_adr.size),
    inputs=[
      m.ngeom,
      m.opt.magnetic,
      m.body_geomnum,
      m.body_geomadr,
      m.body_iquat,
      m.jnt_qposadr,
      m.geom_type,
      m.geom_bodyid,
      m.geom_quat,
      m.site_type,
      m.site_bodyid,
      m.site_size,
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
      m.nxn_pairid,
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
      d.ten_length,
      d.actuator_length,
      rangefinder_dist,
      sensor_collision,
    ],
    outputs=[d.sensordata],
  )

  # jointlimitpos and tendonlimitpos
  wp.launch(
    _limit_pos,
    dim=(d.nworld, d.njmax, m.sensor_limitpos_adr.size),
    inputs=[
      m.sensor_type,
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
def _limit_vel(
  # Model:
  sensor_type: wp.array(dtype=int),
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
    if efc_type == ConstraintType.LIMIT_JOINT or efc_type == ConstraintType.LIMIT_TENDON:
      _write_scalar(
        sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, efc_vel_in[worldid, efcid], sensordata_out[worldid]
      )


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
  if objtype == ObjType.BODY:
    pos = xipos_in[worldid, objid]
    bodyid = objid
  elif objtype == ObjType.XBODY:
    pos = xpos_in[worldid, objid]
    bodyid = objid
  elif objtype == ObjType.GEOM:
    pos = geom_xpos_in[worldid, objid]
    bodyid = geom_bodyid[objid]
  elif objtype == ObjType.SITE:
    pos = site_xpos_in[worldid, objid]
    bodyid = site_bodyid[objid]
  elif objtype == ObjType.CAMERA:
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
  if objtype == ObjType.BODY:
    xpos = xipos_in[worldid, objid]
  elif objtype == ObjType.XBODY:
    xpos = xpos_in[worldid, objid]
  elif objtype == ObjType.GEOM:
    xpos = geom_xpos_in[worldid, objid]
  elif objtype == ObjType.SITE:
    xpos = site_xpos_in[worldid, objid]
  elif objtype == ObjType.CAMERA:
    xpos = cam_xpos_in[worldid, objid]
  else:  # UNKNOWN
    xpos = wp.vec3(0.0)

  if reftype == ObjType.BODY:
    xposref = xipos_in[worldid, refid]
    xmatref = ximat_in[worldid, refid]
  elif reftype == ObjType.XBODY:
    xposref = xpos_in[worldid, refid]
    xmatref = xmat_in[worldid, refid]
  elif reftype == ObjType.GEOM:
    xposref = geom_xpos_in[worldid, refid]
    xmatref = geom_xmat_in[worldid, refid]
  elif reftype == ObjType.SITE:
    xposref = site_xpos_in[worldid, refid]
    xmatref = site_xmat_in[worldid, refid]
  elif reftype == ObjType.CAMERA:
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
    if reftype == ObjType.BODY:
      xmatref = ximat_in[worldid, refid]
    elif reftype == ObjType.XBODY:
      xmatref = xmat_in[worldid, refid]
    elif reftype == ObjType.GEOM:
      xmatref = geom_xmat_in[worldid, refid]
    elif reftype == ObjType.SITE:
      xmatref = site_xmat_in[worldid, refid]
    elif reftype == ObjType.CAMERA:
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

  if sensortype == SensorType.VELOCIMETER:
    vec3 = _velocimeter(body_rootid, site_bodyid, site_xpos_in, site_xmat_in, subtree_com_in, cvel_in, worldid, objid)
    _write_vector(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, vec3, out)
  elif sensortype == SensorType.GYRO:
    vec3 = _gyro(site_bodyid, site_xmat_in, cvel_in, worldid, objid)
    _write_vector(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, vec3, out)
  elif sensortype == SensorType.JOINTVEL:
    val = _joint_vel(jnt_dofadr, qvel_in, worldid, objid)
    _write_scalar(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, val, out)
  elif sensortype == SensorType.TENDONVEL:
    val = _tendon_vel(ten_velocity_in, worldid, objid)
    _write_scalar(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, val, out)
  elif sensortype == SensorType.ACTUATORVEL:
    val = _actuator_vel(actuator_velocity_in, worldid, objid)
    _write_scalar(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, val, out)
  elif sensortype == SensorType.BALLANGVEL:
    vec3 = _ball_ang_vel(jnt_dofadr, qvel_in, worldid, objid)
    _write_vector(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, vec3, out)
  elif sensortype == SensorType.FRAMELINVEL:
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
    _write_vector(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, frame_linvel, out)
  elif sensortype == SensorType.FRAMEANGVEL:
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
    _write_vector(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, frame_angvel, out)
  elif sensortype == SensorType.SUBTREELINVEL:
    vec3 = _subtree_linvel(subtree_linvel_in, worldid, objid)
    _write_vector(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, vec3, out)
  elif sensortype == SensorType.SUBTREEANGMOM:
    vec3 = _subtree_angmom(subtree_angmom_in, worldid, objid)
    _write_vector(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, vec3, out)


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
    _limit_vel,
    dim=(d.nworld, d.njmax, m.sensor_limitvel_adr.size),
    inputs=[
      m.sensor_type,
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

  if actuator_trntype[actid] == TrnType.TENDON and actuator_trnid[actid][0] == sensor_objid[sensorid]:
    adr = sensor_adr[sensorid]
    sensordata_out[worldid, adr] += actuator_force_in[worldid, actid]


@wp.kernel
def _tendon_actuator_force_cutoff(
  # Model:
  sensor_type: wp.array(dtype=int),
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

  _write_scalar(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, val, sensordata_out[worldid])


@wp.kernel
def _limit_frc(
  # Model:
  sensor_type: wp.array(dtype=int),
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
    if efc_type == ConstraintType.LIMIT_JOINT or efc_type == ConstraintType.LIMIT_TENDON:
      _write_scalar(
        sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, efc_force_in[worldid, efcid], sensordata_out[worldid]
      )


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
  if objtype == ObjType.BODY:
    bodyid = objid
    pos = xipos_in[worldid, objid]
  elif objtype == ObjType.XBODY:
    bodyid = objid
    pos = xpos_in[worldid, objid]
  elif objtype == ObjType.GEOM:
    bodyid = geom_bodyid[objid]
    pos = geom_xpos_in[worldid, objid]
  elif objtype == ObjType.SITE:
    bodyid = site_bodyid[objid]
    pos = site_xpos_in[worldid, objid]
  elif objtype == ObjType.CAMERA:
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
  if objtype == ObjType.BODY or objtype == ObjType.XBODY:
    bodyid = objid
  elif objtype == ObjType.GEOM:
    bodyid = geom_bodyid[objid]
  elif objtype == ObjType.SITE:
    bodyid = site_bodyid[objid]
  elif objtype == ObjType.CAMERA:
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
  sensor_adr_to_contact_adr: wp.array(dtype=int),
  # Data in:
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
  cacc_in: wp.array2d(dtype=wp.spatial_vector),
  cfrc_int_in: wp.array2d(dtype=wp.spatial_vector),
  contact_dist_in: wp.array(dtype=float),
  contact_pos_in: wp.array(dtype=wp.vec3),
  contact_frame_in: wp.array(dtype=wp.mat33),
  contact_friction_in: wp.array(dtype=vec5),
  contact_dim_in: wp.array(dtype=int),
  contact_efc_address_in: wp.array2d(dtype=int),
  efc_force_in: wp.array2d(dtype=float),
  njmax_in: int,
  nacon_in: wp.array(dtype=int),
  # In:
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

  if sensortype == SensorType.CONTACT:
    dataspec = sensor_intprm[sensorid, 0]
    dim = sensor_dim[sensorid]
    objtype = sensor_objtype[sensorid]
    reduce = sensor_intprm[sensorid, 1]

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
    contactsensorid = sensor_adr_to_contact_adr[sensorid]
    nmatch = sensor_contact_nmatch_in[worldid, contactsensorid]

    if reduce == 3:  # netforce
      # compute point: force-weighted centroid of contact position
      net_pos = wp.vec3(0.0)
      total_force_magnitude = float(0.0)

      for i in range(nmatch):
        cid = sensor_contact_matchid_in[worldid, contactsensorid, i]

        contact_forcetorque = support.contact_force_fn(
          opt_cone,
          contact_frame_in,
          contact_friction_in,
          contact_dim_in,
          contact_efc_address_in,
          efc_force_in,
          njmax_in,
          nacon_in,
          worldid,
          cid,
          False,
        )

        weight = wp.norm_l2(wp.spatial_top(contact_forcetorque))
        net_pos += weight * contact_pos_in[cid]
        total_force_magnitude += weight

      net_pos /= wp.max(total_force_magnitude, MJ_MINVAL)

      # TODO(team): iterate over matches once

      # compute total wrench about point, in the global frame
      net_force = wp.vec3(0.0)
      net_torque = wp.vec3(0.0)

      for i in range(nmatch):
        cid = sensor_contact_matchid_in[worldid, contactsensorid, i]
        dir = sensor_contact_direction_in[worldid, contactsensorid, i]

        contact_forcetorque = support.contact_force_fn(
          opt_cone,
          contact_frame_in,
          contact_friction_in,
          contact_dim_in,
          contact_efc_address_in,
          efc_force_in,
          njmax_in,
          nacon_in,
          worldid,
          cid,
          False,
        )
        contact_forcetorque *= dir

        force_local = wp.spatial_top(contact_forcetorque)
        torque_local = wp.spatial_bottom(contact_forcetorque)

        frame = contact_frame_in[cid]
        frameT = wp.transpose(frame)

        force_global = frameT @ force_local
        torque_global = frameT @ torque_local

        # add to total force, torque
        net_force += force_global
        net_torque += torque_global

        # add induced moment: torque += (pos - point) x force
        net_torque += wp.cross(contact_pos_in[cid] - net_pos, force_global)

      adr_slot = adr

      if found:
        out[adr_slot] = float(nmatch)
        adr_slot += 1
      if force:
        out[adr_slot + 0] = net_force[0]
        out[adr_slot + 1] = net_force[1]
        out[adr_slot + 2] = net_force[2]
        adr_slot += 3
      if torque:
        out[adr_slot + 0] = net_torque[0]
        out[adr_slot + 1] = net_torque[1]
        out[adr_slot + 2] = net_torque[2]
        adr_slot += 3
      if dist:
        out[adr_slot] = 0.0
        adr_slot += 1
      if pos:
        out[adr_slot + 0] = net_pos[0]
        out[adr_slot + 1] = net_pos[1]
        out[adr_slot + 2] = net_pos[2]
        adr_slot += 3
      if normal:
        out[adr_slot + 0] = 1.0
        out[adr_slot + 1] = 0.0
        out[adr_slot + 2] = 0.0
        adr_slot += 3
      if tangent:
        out[adr_slot + 0] = 0.0
        out[adr_slot + 1] = 1.0
        out[adr_slot + 2] = 0.0
    else:
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
            contact_frame_in,
            contact_friction_in,
            contact_dim_in,
            contact_efc_address_in,
            efc_force_in,
            njmax_in,
            nacon_in,
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

      # zero remaining slots
      for i in range(nmatch, num):
        for j in range(size):
          out[adr + i * size + j] = 0.0

  elif sensortype == SensorType.ACCELEROMETER:
    vec3 = _accelerometer(
      body_rootid, site_bodyid, site_xpos_in, site_xmat_in, subtree_com_in, cvel_in, cacc_in, worldid, objid
    )
    _write_vector(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, vec3, out)
  elif sensortype == SensorType.FORCE:
    vec3 = _force(site_bodyid, site_xmat_in, cfrc_int_in, worldid, objid)
    _write_vector(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, vec3, out)
  elif sensortype == SensorType.TORQUE:
    vec3 = _torque(body_rootid, site_bodyid, site_xpos_in, site_xmat_in, subtree_com_in, cfrc_int_in, worldid, objid)
    _write_vector(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, vec3, out)
  elif sensortype == SensorType.ACTUATORFRC:
    val = _actuator_force(actuator_force_in, worldid, objid)
    _write_scalar(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, val, out)
  elif sensortype == SensorType.JOINTACTFRC:
    val = _joint_actuator_force(jnt_dofadr, qfrc_actuator_in, worldid, objid)
    _write_scalar(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, val, out)
  elif sensortype == SensorType.FRAMELINACC:
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
    _write_vector(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, vec3, out)
  elif sensortype == SensorType.FRAMEANGACC:
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
    _write_vector(sensor_type, sensor_datatype, sensor_adr, sensor_cutoff, sensorid, 3, vec3, out)


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
  site_xpos_in: wp.array2d(dtype=wp.vec3),
  site_xmat_in: wp.array2d(dtype=wp.mat33),
  contact_pos_in: wp.array(dtype=wp.vec3),
  contact_frame_in: wp.array(dtype=wp.mat33),
  contact_dim_in: wp.array(dtype=int),
  contact_geom_in: wp.array(dtype=wp.vec2i),
  contact_efc_address_in: wp.array2d(dtype=int),
  contact_worldid_in: wp.array(dtype=int),
  efc_force_in: wp.array2d(dtype=float),
  nacon_in: wp.array(dtype=int),
  # Data out:
  sensordata_out: wp.array2d(dtype=float),
):
  conid, sensortouchadrid = wp.tid()

  if conid >= nacon_in[0]:
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

    if opt_cone == ConeType.PYRAMIDAL:
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


@wp.func
def _transform_spatial(vec: wp.spatial_vector, dif: wp.vec3) -> wp.vec3:
  return wp.spatial_bottom(vec) - wp.cross(dif, wp.spatial_top(vec))


@wp.kernel
def _sensor_tactile(
  # Model:
  body_rootid: wp.array(dtype=int),
  body_weldid: wp.array(dtype=int),
  oct_child: wp.array(dtype=vec8i),
  oct_aabb: wp.array2d(dtype=wp.vec3),
  oct_coeff: wp.array(dtype=vec8),
  geom_type: wp.array(dtype=int),
  geom_bodyid: wp.array(dtype=int),
  geom_size: wp.array2d(dtype=wp.vec3),
  mesh_vertadr: wp.array(dtype=int),
  mesh_normaladr: wp.array(dtype=int),
  mesh_vert: wp.array(dtype=wp.vec3),
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
  geom_xpos_in: wp.array2d(dtype=wp.vec3),
  geom_xmat_in: wp.array2d(dtype=wp.mat33),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  cvel_in: wp.array2d(dtype=wp.spatial_vector),
  contact_geom_in: wp.array(dtype=wp.vec2i),
  contact_worldid_in: wp.array(dtype=int),
  nacon_in: wp.array(dtype=int),
  # Data out:
  sensordata_out: wp.array2d(dtype=float),
):
  conid, taxelid = wp.tid()

  if conid >= nacon_in[0]:
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

  plugin_id = geom_plugin_index[geom]

  contact_type = geom_type[geom]

  plugin_attributes, plugin_index, volume_data, mesh_data = get_sdf_params(
    oct_child, oct_aabb, oct_coeff, plugin, plugin_attr, contact_type, geom_size[worldid, geom], plugin_id, mesh_id
  )

  depth = wp.min(sdf(contact_type, lpos, plugin_attributes, plugin_index, volume_data, mesh_data), 0.0)
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


@wp.func
def _check_match(body_parentid: wp.array(dtype=int), body: int, geom: int, objtype: int, objid: int) -> bool:
  """Check if a contact body/geom matches a sensor spec (objtype, objid)."""
  if objtype == ObjType.UNKNOWN:
    return True
  if objtype == ObjType.SITE:
    return True  # already passed site filter test
  if objtype == ObjType.GEOM:
    return objid == geom
  if objtype == ObjType.BODY:
    return objid == body
  if objtype == ObjType.XBODY:
    # traverse up the tree from body, return true if we land on id
    while body > objid:
      body = body_parentid[body]
    return body == objid
  return False


@wp.kernel
def _contact_match(
  # Model:
  opt_cone: int,
  opt_contact_sensor_maxmatch: int,
  body_parentid: wp.array(dtype=int),
  geom_bodyid: wp.array(dtype=int),
  site_type: wp.array(dtype=int),
  site_size: wp.array(dtype=wp.vec3),
  sensor_objtype: wp.array(dtype=int),
  sensor_objid: wp.array(dtype=int),
  sensor_reftype: wp.array(dtype=int),
  sensor_refid: wp.array(dtype=int),
  sensor_intprm: wp.array2d(dtype=int),
  sensor_contact_adr: wp.array(dtype=int),
  # Data in:
  site_xpos_in: wp.array2d(dtype=wp.vec3),
  site_xmat_in: wp.array2d(dtype=wp.mat33),
  contact_dist_in: wp.array(dtype=float),
  contact_pos_in: wp.array(dtype=wp.vec3),
  contact_frame_in: wp.array(dtype=wp.mat33),
  contact_friction_in: wp.array(dtype=vec5),
  contact_dim_in: wp.array(dtype=int),
  contact_geom_in: wp.array(dtype=wp.vec2i),
  contact_efc_address_in: wp.array2d(dtype=int),
  contact_worldid_in: wp.array(dtype=int),
  contact_type_in: wp.array(dtype=int),
  efc_force_in: wp.array2d(dtype=float),
  njmax_in: int,
  nacon_in: wp.array(dtype=int),
  # Out:
  sensor_contact_nmatch_out: wp.array2d(dtype=int),
  sensor_contact_matchid_out: wp.array3d(dtype=int),
  sensor_contact_criteria_out: wp.array3d(dtype=float),
  sensor_contact_direction_out: wp.array3d(dtype=float),
):
  contactsensorid, contactid = wp.tid()
  sensorid = sensor_contact_adr[contactsensorid]

  if contactid >= nacon_in[0]:
    return

  if not contact_type_in[contactid] & ContactType.CONSTRAINT:
    return

  # sensor information
  objtype = sensor_objtype[sensorid]
  objid = sensor_objid[sensorid]
  reftype = sensor_reftype[sensorid]
  refid = sensor_refid[sensorid]
  reduce = sensor_intprm[sensorid, 1]

  worldid = contact_worldid_in[contactid]

  # site filter
  if objtype == ObjType.SITE:
    if not inside_geom(
      site_xpos_in[worldid, objid], site_xmat_in[worldid, objid], site_size[objid], site_type[objid], contact_pos_in[contactid]
    ):
      return

  # unknown-unknown match
  if objtype == ObjType.UNKNOWN and reftype == ObjType.UNKNOWN:
    dir = 1.0
  else:
    # contact information
    geom = contact_geom_in[contactid]
    geom1 = geom[0]
    geom2 = geom[1]
    body1 = geom_bodyid[geom1]
    body2 = geom_bodyid[geom2]

    # check match of sensor objects with contact objects
    match11 = _check_match(body_parentid, body1, geom1, objtype, objid)
    match12 = _check_match(body_parentid, body2, geom2, objtype, objid)
    match21 = _check_match(body_parentid, body1, geom1, reftype, refid)
    match22 = _check_match(body_parentid, body2, geom2, reftype, refid)

    # if a sensor object is specified, it must be involved in the contact
    if not match11 and not match12:
      return
    if not match21 and not match22:
      return

    # determine direction
    dir = 1.0
    if objtype != ObjType.UNKNOWN and reftype != ObjType.UNKNOWN:
      # both obj1 and obj2 specified: direction depends on order
      order_regular = match11 and match22
      order_reverse = match12 and match21
      if not order_regular and not order_reverse:
        return
      if order_reverse and not order_regular:
        dir = -1.0
    elif objtype != ObjType.UNKNOWN:
      if not match11:
        dir = -1.0
    elif reftype != ObjType.UNKNOWN:
      if not match22:
        dir = -1.0

  contactmatchid = wp.atomic_add(sensor_contact_nmatch_out[worldid], contactsensorid, 1)

  if contactmatchid >= opt_contact_sensor_maxmatch:
    # TODO(team): alternative to wp.printf for reporting overflow?
    wp.printf("contact match overflow: please increase Option.contact_sensor_maxmatch to %u\n", contactmatchid)
    return

  sensor_contact_matchid_out[worldid, contactsensorid, contactmatchid] = contactid

  if reduce == 1:  # mindist
    sensor_contact_criteria_out[worldid, contactsensorid, contactmatchid] = contact_dist_in[contactid]
  elif reduce == 2:  # maxforce
    contact_force = support.contact_force_fn(
      opt_cone,
      contact_frame_in,
      contact_friction_in,
      contact_dim_in,
      contact_efc_address_in,
      efc_force_in,
      njmax_in,
      nacon_in,
      worldid,
      contactid,
      False,
    )
    force_magnitude = (
      contact_force[0] * contact_force[0] + contact_force[1] * contact_force[1] + contact_force[2] * contact_force[2]
    )
    sensor_contact_criteria_out[worldid, contactsensorid, contactmatchid] = -force_magnitude

  # contact direction
  sensor_contact_direction_out[worldid, contactsensorid, contactmatchid] = dir


@cache_kernel
def _contact_sort(maxmatch: int):
  @nested_kernel(module="unique", enable_backward=False)
  def contact_sort(
    # Model:
    sensor_intprm: wp.array2d(dtype=int),
    sensor_contact_adr: wp.array(dtype=int),
    # Data in:
    sensor_contact_nmatch_in: wp.array2d(dtype=int),
    sensor_contact_matchid_in: wp.array3d(dtype=int),
    sensor_contact_criteria_in: wp.array3d(dtype=float),
    # Data out:
    sensor_contact_matchid_out: wp.array3d(dtype=int),
  ):
    worldid, contactsensorid = wp.tid()

    worldid, contactsensorid = wp.tid()
    sensorid = sensor_contact_adr[contactsensorid]

    reduce = sensor_intprm[sensorid, 1]
    if reduce == 0 or reduce == 3:  # none or netforce
      return

    nmatch = sensor_contact_nmatch_in[worldid, contactsensorid]

    # skip sort
    if nmatch <= 1:
      return

    criteria_tile = wp.tile_load(sensor_contact_criteria_in[worldid, contactsensorid], shape=maxmatch)
    matchid_tile = wp.tile_load(sensor_contact_matchid_in[worldid, contactsensorid], shape=maxmatch)
    wp.tile_sort(criteria_tile, matchid_tile)
    wp.tile_store(sensor_contact_matchid_out[worldid, contactsensorid], matchid_tile)

  return contact_sort


@event_scope
def sensor_acc(m: Model, d: Data):
  """Compute acceleration-dependent sensor values."""
  if m.opt.disableflags & DisableBit.SENSOR:
    return

  wp.launch(
    _sensor_touch,
    dim=(d.naconmax, m.sensor_touch_adr.size),
    inputs=[
      m.opt.cone,
      m.geom_bodyid,
      m.site_type,
      m.site_bodyid,
      m.site_size,
      m.sensor_objid,
      m.sensor_adr,
      m.sensor_touch_adr,
      d.site_xpos,
      d.site_xmat,
      d.contact.pos,
      d.contact.frame,
      d.contact.dim,
      d.contact.geom,
      d.contact.efc_address,
      d.contact.worldid,
      d.efc.force,
      d.nacon,
    ],
    outputs=[
      d.sensordata,
    ],
  )

  wp.launch(
    _sensor_tactile,
    dim=(d.naconmax, m.nsensortaxel),
    inputs=[
      m.body_rootid,
      m.body_weldid,
      m.oct_child,
      m.oct_aabb,
      m.oct_coeff,
      m.geom_type,
      m.geom_bodyid,
      m.geom_size,
      m.mesh_vertadr,
      m.mesh_normaladr,
      m.mesh_vert,
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
      d.geom_xpos,
      d.geom_xmat,
      d.subtree_com,
      d.cvel,
      d.contact.geom,
      d.contact.worldid,
      d.nacon,
    ],
    outputs=[
      d.sensordata,
    ],
  )

  sensor_contact_nmatch = wp.empty((d.nworld, m.nsensorcontact), dtype=int)
  sensor_contact_matchid = wp.empty((d.nworld, m.nsensorcontact, m.opt.contact_sensor_maxmatch), dtype=int)
  sensor_contact_direction = wp.empty((d.nworld, m.nsensorcontact, m.opt.contact_sensor_maxmatch), dtype=float)
  if m.nsensorcontact:
    sensor_contact_criteria = wp.empty((d.nworld, m.nsensorcontact, m.opt.contact_sensor_maxmatch), dtype=float)
    # TODO(team): fill_ operations in one kernel?
    sensor_contact_nmatch.fill_(0)
    sensor_contact_matchid.fill_(-1)
    sensor_contact_criteria.fill_(1.0e32)

    wp.launch(
      _contact_match,
      dim=(m.sensor_contact_adr.size, d.naconmax),
      inputs=[
        m.opt.cone,
        m.opt.contact_sensor_maxmatch,
        m.body_parentid,
        m.geom_bodyid,
        m.site_type,
        m.site_size,
        m.sensor_objtype,
        m.sensor_objid,
        m.sensor_reftype,
        m.sensor_refid,
        m.sensor_intprm,
        m.sensor_contact_adr,
        d.site_xpos,
        d.site_xmat,
        d.contact.dist,
        d.contact.pos,
        d.contact.frame,
        d.contact.friction,
        d.contact.dim,
        d.contact.geom,
        d.contact.efc_address,
        d.contact.worldid,
        d.contact.type,
        d.efc.force,
        d.njmax,
        d.nacon,
      ],
      outputs=[sensor_contact_nmatch, sensor_contact_matchid, sensor_contact_criteria, sensor_contact_direction],
    )

    # sorting
    wp.launch_tiled(
      _contact_sort(m.opt.contact_sensor_maxmatch),
      dim=(d.nworld, m.sensor_contact_adr.size),
      inputs=[m.sensor_intprm, m.sensor_contact_adr, sensor_contact_nmatch, sensor_contact_matchid, sensor_contact_criteria],
      outputs=[sensor_contact_matchid],
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
      m.sensor_adr_to_contact_adr,
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
      d.cacc,
      d.cfrc_int,
      d.contact.dist,
      d.contact.pos,
      d.contact.frame,
      d.contact.friction,
      d.contact.dim,
      d.contact.efc_address,
      d.efc.force,
      d.njmax,
      d.nacon,
      sensor_contact_nmatch,
      sensor_contact_matchid,
      sensor_contact_direction,
    ],
    outputs=[d.sensordata],
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
      m.sensor_type,
      m.sensor_datatype,
      m.sensor_adr,
      m.sensor_cutoff,
      m.sensor_tendonactfrc_adr,
      d.sensordata,
    ],
    outputs=[d.sensordata],
  )

  wp.launch(
    _limit_frc,
    dim=(d.nworld, d.njmax, m.sensor_limitfrc_adr.size),
    inputs=[
      m.sensor_type,
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
  gravity = opt_gravity[worldid % opt_gravity.shape[0]]
  bodyid += 1  # skip world body

  energy = wp.vec2(
    body_mass[worldid % body_mass.shape[0], bodyid] * wp.dot(gravity, xipos_in[worldid, bodyid]),
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
  jnt_stiffness_id = worldid % jnt_stiffness.shape[0]
  stiffness = jnt_stiffness[jnt_stiffness_id, jntid]

  if stiffness == 0.0:
    return

  padr = jnt_qposadr[jntid]
  jnttype = jnt_type[jntid]
  qpos_spring_id = worldid % qpos_spring.shape[0]

  if jnttype == JointType.FREE:
    dif0 = wp.vec3(
      qpos_in[worldid, padr + 0] - qpos_spring[qpos_spring_id, padr + 0],
      qpos_in[worldid, padr + 1] - qpos_spring[qpos_spring_id, padr + 1],
      qpos_in[worldid, padr + 2] - qpos_spring[qpos_spring_id, padr + 2],
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
      qpos_spring[qpos_spring_id, padr + 3],
      qpos_spring[qpos_spring_id, padr + 4],
      qpos_spring[qpos_spring_id, padr + 5],
      qpos_spring[qpos_spring_id, padr + 6],
    )

    dif1 = math.quat_sub(quat1, quat_spring)

    energy = wp.vec2(
      0.5 * stiffness * (wp.dot(dif0, dif0) + wp.dot(dif1, dif1)),
      0.0,
    )

    wp.atomic_add(energy_out, worldid, energy)

  elif jnttype == JointType.BALL:
    quat = wp.quat(
      qpos_in[worldid, padr + 0],
      qpos_in[worldid, padr + 1],
      qpos_in[worldid, padr + 2],
      qpos_in[worldid, padr + 3],
    )
    quat = wp.normalize(quat)

    quat_spring = wp.quat(
      qpos_spring[qpos_spring_id, padr + 0],
      qpos_spring[qpos_spring_id, padr + 1],
      qpos_spring[qpos_spring_id, padr + 2],
      qpos_spring[qpos_spring_id, padr + 3],
    )

    dif = math.quat_sub(quat, quat_spring)
    energy = wp.vec2(
      0.5 * stiffness * wp.dot(dif, dif),
      0.0,
    )
    wp.atomic_add(energy_out, worldid, energy)
  elif jnttype == JointType.SLIDE or jnttype == JointType.HINGE:
    dif_ = qpos_in[worldid, padr] - qpos_spring[qpos_spring_id, padr]
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

  tendon_stiffness_id = worldid % tendon_stiffness.shape[0]
  stiffness = tendon_stiffness[tendon_stiffness_id, tenid]

  if stiffness == 0.0:
    return

  length = ten_length_in[worldid, tenid]

  # compute spring displacement
  tendon_lengthspring_id = worldid % tendon_lengthspring.shape[0]
  lengthspring = tendon_lengthspring[tendon_lengthspring_id, tenid]
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
  wp.launch(_energy_pos_zero, dim=d.nworld, outputs=[d.energy])

  # init potential energy: -sum_i(body_i.mass * dot(gravity, body_i.pos))
  if not m.opt.disableflags & DisableBit.GRAVITY:
    wp.launch(
      _energy_pos_gravity, dim=(d.nworld, m.nbody - 1), inputs=[m.opt.gravity, m.body_mass, d.xipos], outputs=[d.energy]
    )

  if not m.opt.disableflags & DisableBit.SPRING:
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
  @nested_kernel(module="unique", enable_backward=False)
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
  support.mul_m(m, d, d.efc.mv, d.qvel)

  wp.launch_tiled(
    _energy_vel_kinetic(m.nv),
    dim=d.nworld,
    inputs=[d.qvel, d.efc.mv],
    outputs=[d.energy],
    block_dim=m.block_dim.energy_vel_kinetic,
  )
