from typing import List, Optional, Tuple

import utils as utils_component

import numpy as np

from pxr import Gf
from pxr import Usd
from pxr import UsdGeom

class USDCamera:

  def __init__(self, stage: Usd.Stage, obj_name: str):
    self.stage = stage

    xform_path = f"/World/Camera_Xform_{obj_name}"
    camera_path = f"{xform_path}/Camera_{obj_name}"
    self.usd_xform = UsdGeom.Xform.Define(stage, xform_path)
    self.usd_camera = UsdGeom.Camera.Define(stage, camera_path)
    self.usd_prim = stage.GetPrimAtPath(camera_path)

    # defining ops required by update function
    self.transform_op = self.usd_xform.AddTransformOp()

    # self.usd_camera.CreateFocalLengthAttr().Set(18.14756) # default in omniverse
    self.usd_camera.CreateFocalLengthAttr().Set(12)
    self.usd_camera.CreateFocusDistanceAttr().Set(400)

    self.usd_camera.GetHorizontalApertureAttr().Set(12)

    self.usd_camera.GetClippingRangeAttr().Set(Gf.Vec2f(1e-4, 1e6))

  def update(self, cam_pos: np.ndarray, cam_mat: np.ndarray, frame: int):

    transformation_mat = utils_component.create_transform_matrix(
        rotation_matrix=cam_mat, translation_vector=cam_pos
    ).T
    self.transform_op.Set(Gf.Matrix4d(transformation_mat.tolist()), frame)