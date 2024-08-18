# Copyright 2023 DeepMind Technologies Limited
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
"""Sensor functions."""

import jax
from jax import numpy as jp
import mujoco
# pylint: disable=g-importing-member
from mujoco.mjx._src import math
from mujoco.mjx._src.types import Data
from mujoco.mjx._src.types import DisableBit
from mujoco.mjx._src.types import Model
from mujoco.mjx._src.types import ObjType
from mujoco.mjx._src.types import SensorType
# pylint: enable=g-importing-member
import numpy as np


def sensor_pos(m: Model, d: Data) -> Data:
  """Compute position-dependent sensors values."""

  if m.opt.disableflags & DisableBit.SENSOR:
    return d

  # no position-dependent sensors
  stage_pos = m.sensor_needstage == mujoco.mjtStage.mjSTAGE_POS
  if sum(stage_pos) == 0:
    return d

  # position and orientation by object type
  objtype_data = {
      ObjType.UNKNOWN: (
          np.expand_dims(np.eye(3), axis=0),
          np.zeros((1, 3)),
      ),  # world
      ObjType.BODY: (d.xipos, d.ximat),
      ObjType.XBODY: (d.xpos, d.xmat),
      ObjType.GEOM: (d.geom_xpos, d.geom_xmat),
      ObjType.SITE: (d.site_xpos, d.site_xmat),
      ObjType.CAMERA: (d.cam_xpos, d.cam_xmat),
  }

  # frame axis indexing
  frame_axis = {
      SensorType.FRAMEXAXIS: 0,
      SensorType.FRAMEYAXIS: 1,
      SensorType.FRAMEZAXIS: 2,
  }

  sensors, adrs = [], []

  for sensor_type in set(m.sensor_type[stage_pos]):
    idx = m.sensor_type == sensor_type
    objid = m.sensor_objid[idx]
    adr = m.sensor_adr[idx]

    if sensor_type == SensorType.MAGNETOMETER:
      sensor = jax.vmap(lambda xmat: xmat.T @ m.opt.magnetic)(
          d.site_xmat[objid]
      ).reshape(-1)
      adr = (adr[:, None] + np.arange(3)[None]).reshape(-1)
    elif sensor_type == SensorType.JOINTPOS:
      sensor = d.qpos[m.jnt_qposadr[objid]]
    elif sensor_type == SensorType.ACTUATORPOS:
      sensor = d.actuator_length[objid]
    elif sensor_type == SensorType.BALLQUAT:
      jnt_qposadr = m.jnt_qposadr[objid, None] + np.arange(4)[None]
      quat = d.qpos[jnt_qposadr]
      sensor = jax.vmap(math.normalize)(quat).reshape(-1)
      adr = (adr[:, None] + np.arange(4)[None]).reshape(-1)
    elif sensor_type == SensorType.FRAMEPOS:

      def _framepos(xpos, xpos_ref, xmat_ref, refid):
        return jp.where(refid == -1, xpos, xmat_ref.T @ (xpos - xpos_ref))

      objtype = m.sensor_objtype[idx]
      reftype = m.sensor_reftype[idx]
      refid = m.sensor_refid[idx]

      # evaluate for valid object and reference object type pairs
      for ot, rt in set(zip(objtype, reftype)):
        id_ = (objtype == ot) & (reftype == rt)
        refid_ = refid[id_]
        xpos, _ = objtype_data[ot]
        xpos_ref, xmat_ref = objtype_data[rt]
        xpos = xpos[objid[id_]]
        xpos_ref = xpos_ref[refid_]
        xmat_ref = xmat_ref[refid_]
        sensor = jax.vmap(_framepos)(xpos, xpos_ref, xmat_ref, refid_)
        adr_ = adr[id_, None] + np.arange(3)[None]
        sensors.append(sensor.reshape(-1))
        adrs.append(adr_.reshape(-1))
      continue  # avoid adding to sensors/adrs list a second time
    elif sensor_type in frame_axis:

      def _frameaxis(xmat, xmat_ref, refid):
        axis = xmat[:, frame_axis[sensor_type]]
        return jp.where(refid == -1, axis, xmat_ref.T @ axis)

      objtype = m.sensor_objtype[idx]
      reftype = m.sensor_reftype[idx]
      refid = m.sensor_refid[idx]

      # evaluate for valid object and reference object type pairs
      for ot, rt in set(zip(objtype, reftype)):
        id_ = (objtype == ot) & (reftype == rt)
        refid_ = refid[id_]
        _, xmat = objtype_data[ot]
        _, xmat_ref = objtype_data[rt]
        xmat = xmat[objid[id_]]
        xmat_ref = xmat_ref[refid_]
        sensor = jax.vmap(_frameaxis)(xmat, xmat_ref, refid_)
        adr_ = adr[id_, None] + np.arange(3)[None]
        sensors.append(sensor.reshape(-1))
        adrs.append(adr_.reshape(-1))
      continue  # avoid adding to sensors/adrs list a second time
    elif sensor_type == SensorType.SUBTREECOM:
      sensor = d.subtree_com[objid].reshape(-1)
      adr = (adr[:, None] + np.arange(3)[None]).reshape(-1)
    elif sensor_type == SensorType.CLOCK:
      sensor = jp.repeat(d.time, sum(idx))
    else:
      continue  # unsupported sensor type

    sensors.append(sensor)
    adrs.append(adr)

  if not adrs:
    return d

  sensordata = d.sensordata.at[np.concatenate(adrs)].set(
      jp.concatenate(sensors)
  )

  return d.replace(sensordata=sensordata)


def sensor_vel(m: Model, d: Data) -> Data:
  """Compute velocity-dependent sensors values."""

  if m.opt.disableflags & DisableBit.SENSOR:
    return d

  return d


def sensor_acc(m: Model, d: Data) -> Data:
  """Compute acceleration/force-dependent sensors values."""

  if m.opt.disableflags & DisableBit.SENSOR:
    return d

  return d
