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
"""Utility functions for SDF collision handling."""

import enum
from typing import Dict

import mujoco
import warp as wp

from .bolt import bolt
from .bolt import bolt_sdf_grad
from .gear import gear
from .gear import gear_sdf_grad
from .nut import nut
from .nut import nut_sdf_grad
from .torus import torus
from .torus import torus_sdf_grad


class SDFType(enum.Enum):
  """Enum for SDF types."""

  NUT = "NUT"
  BOLT = "BOLT"
  TORUS = "TORUS"
  GEAR = "GEAR"


def register_sdf_plugins(mjwarp) -> Dict[str, int]:
  xml = """<mujoco>
        <extension>
        <plugin plugin="mujoco.sdf.nut"><instance name="n"/></plugin>
        <plugin plugin="mujoco.sdf.bolt"><instance name="b"/></plugin>
        <plugin plugin="mujoco.sdf.torus"><instance name="t"/></plugin>
        <plugin plugin="mujoco.sdf.gear"><instance name="g"/></plugin>
        </extension>
        <asset>
        <mesh name="nm"><plugin instance="n"/></mesh>
        <mesh name="bm"><plugin instance="b"/></mesh>
        <mesh name="tm"><plugin instance="t"/></mesh>
        <mesh name="gm"><plugin instance="g"/></mesh>
        </asset>
        <worldbody>
        <body><geom type="sdf" name="ng" mesh="nm"><plugin instance="n"/></geom></body>
        <body><geom type="sdf" name="bg" mesh="bm"><plugin instance="b"/></geom></body>
        <body><geom type="sdf" name="tg" mesh="tm"><plugin instance="t"/></geom></body>
        <body><geom type="sdf" name="gg" mesh="gm"><plugin instance="g"/></geom></body>
        </worldbody>
        </mujoco>"""

  try:
    m = mujoco.MjModel.from_xml_string(xml)
  except Exception as e:
    raise ValueError(f"Failed to create MuJoCo model from XML: {e}")

  sdf_types = {}

  for i in range(m.ngeom):
    name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i)
    if name == "ng":
      sdf_types[SDFType.NUT.value] = int(m.plugin[i])
    elif name == "bg":
      sdf_types[SDFType.BOLT.value] = int(m.plugin[i])
    elif name == "tg":
      sdf_types[SDFType.TORUS.value] = int(m.plugin[i])

    elif name == "gg":
      sdf_types[SDFType.GEAR.value] = int(m.plugin[i])

  @wp.func
  def user_sdf(p: wp.vec3, attr: wp.vec3, sdf_type: int) -> float:
    result = 0.0
    if sdf_type == wp.static(sdf_types[SDFType.NUT.value]):
      result = nut(p, attr)
    elif sdf_type == wp.static(sdf_types[SDFType.BOLT.value]):
      result = bolt(p, attr)
    elif sdf_type == wp.static(sdf_types[SDFType.TORUS.value]):
      result = torus(p, attr)
    elif sdf_type == wp.static(sdf_types[SDFType.GEAR.value]):
      result = gear(p, attr)
    return result

  @wp.func
  def user_sdf_grad(p: wp.vec3, attr: wp.vec3, sdf_type: int) -> wp.vec3:
    if sdf_type == wp.static(sdf_types[SDFType.NUT.value]):
      return nut_sdf_grad(p, attr)
    elif sdf_type == wp.static(sdf_types[SDFType.BOLT.value]):
      return bolt_sdf_grad(p, attr)
    elif sdf_type == wp.static(sdf_types[SDFType.TORUS.value]):
      return torus_sdf_grad(p, attr)
    elif sdf_type == wp.static(sdf_types[SDFType.GEAR.value]):
      return gear_sdf_grad(p, attr)
    return wp.vec3()

  mjwarp._src.collision_sdf.user_sdf = user_sdf
  mjwarp._src.collision_sdf.user_sdf_grad = user_sdf_grad

  return sdf_types
