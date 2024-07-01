from typing import List, Optional, Tuple

import numpy as np

from pxr import Gf
from pxr import Usd
from pxr import UsdGeom
from pxr import UsdLux

class USDSphereLight:

  def __init__(
      self, stage: Usd.Stage, obj_name: str, radius: Optional[float] = 0.3
  ):
    self.stage = stage

    xform_path = f"/World/Light_Xform_{obj_name}"
    light_path = f"{xform_path}/Light_{obj_name}"
    self.usd_xform = UsdGeom.Xform.Define(stage, xform_path)
    self.usd_light = UsdLux.SphereLight.Define(stage, light_path)
    self.usd_prim = stage.GetPrimAtPath(light_path)

    # we assume in mujoco that all lights are point lights
    self.usd_light.GetRadiusAttr().Set(radius)
    self.usd_light.GetTreatAsPointAttr().Set(False)
    self.usd_light.GetNormalizeAttr().Set(True)

    # defining ops required by update function
    self.translate_op = self.usd_xform.AddTranslateOp()

  def update(self, pos: np.ndarray, intensity: int, color: np.ndarray, frame: int):
    self.translate_op.Set(Gf.Vec3d(pos.tolist()), frame)

    if not np.any(pos):
      intensity = 0

    self.usd_light.GetIntensityAttr().Set(intensity)
    self.usd_light.GetColorAttr().Set(Gf.Vec3d(color.tolist()))


class USDDomeLight:

  def __init__(self, stage: Usd.Stage, obj_name: str):
    self.stage = stage

    xform_path = f"/World/Light_Xform_{obj_name}"
    light_path = f"{xform_path}/Light_{obj_name}"
    self.usd_xform = UsdGeom.Xform.Define(stage, xform_path)
    self.usd_light = UsdLux.DomeLight.Define(stage, light_path)
    self.usd_prim = stage.GetPrimAtPath(light_path)

    # we assume in mujoco that all lights are point lights
    self.usd_light.GetNormalizeAttr().Set(True)

  def update(self, intensity: int, color: np.ndarray, frame: int):
    self.usd_light.GetIntensityAttr().Set(intensity)
    self.usd_light.GetExposureAttr().Set(0.0)
    self.usd_light.GetColorAttr().Set(Gf.Vec3d(color.tolist()))

