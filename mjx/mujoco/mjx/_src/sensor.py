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
# pylint: disable=g-importing-member
from mujoco.mjx._src.types import Data
from mujoco.mjx._src.types import Model
from mujoco.mjx._src.types import SensorType
from typing import Tuple
# pylint: enable=g-importing-member
import numpy as np


def sensor_pos(m: Model, d: Data) -> Data:
  """Compute position-dependent sensors values."""

  sensordata = d.sensordata
  if np.isin(SensorType.JOINTPOS, m.sensor_type):
    # jointpos
    i = m.sensor_type == SensorType.JOINTPOS
    objid = m.sensor_objid[i]
    adr = m.sensor_adr[i]
    sensordata = sensordata.at[adr].set(d.qpos[m.jnt_qposadr[objid]])
  if np.isin(SensorType.ACTUATORPOS, m.sensor_type):
    # actuatorpos
    i = m.sensor_type == SensorType.ACTUATORPOS
    objid = m.sensor_objid[i]
    adr = m.sensor_adr[i]
    sensordata = sensordata.at[adr].set(d.actuator_length[objid])

  return d.replace(sensordata=sensordata)


def sensor_vel(m: Model, d: Data) -> Data:
  """Compute velocity-dependent sensors values."""
  return d


def sensor_acc(m: Model, d: Data) -> Data:
  """Compute acceleration/force-dependent sensors values."""
  return d
