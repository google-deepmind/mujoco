from enum import Enum

import mujoco
from pxr import Usd, UsdGeom, Vt

class USDGeomType(Enum):
  """
  Represents different types of geoms we can add to USD
  The values match those found by the enum presented here:
  https://mujoco.readthedocs.io/en/latest/APIreference/APItypes.html#mjtgeom
  """
  Plane = 0
  Sphere = 2
  Cube = 6

class USDGeom(object):
  """
  Parent class for created geoms
  """
  def __init__(self,
               geom=None,
               stage=None):
    self.geom = geom
    self.stage = stage # TODO: remove, not being used
    self.type = None
    self.xform = None
    self.prim = None
    self.ref = None

  def update_geom(self, new_geom):
    raise NotImplementedError
  
  def update_pos(self, new_pos):
    pos = tuple([float(x) for x in new_pos])
    self.xform.AddTranslateOp().Set(pos)

  def update_size(self, new_size):
    size = tuple([float(x) for x in new_size])
    self.xform.AddScaleOp().Set(value=size)

  def update_color(self, new_color):
    # new_color is the rgba (we extract first three)
    rgba = [(float(x) for x in new_color[:3])]
    color = self.prim.GetDisplayColorAttr()
    color.Set(rgba)

  # TODO: create another method for transparency
  def update_transparency(self, new_transparency):
    pass

  def __str__(self):
    return f'type = {self.type} \ngeom = {self.geom}'
  
class USDPlane(USDGeom):
  """
  Stores information regarding a plane geom in USD
  """

  plane_count = 0

  def __init__(self, 
               geom=None,
               stage=None):
    super().__init__(geom, stage)
    self.type = 0
    USDPlane.plane_count += 1
    xform_path = f'/Plane_Xform_{USDPlane.plane_count}'
    plane_path = f'{xform_path}/Plane_{USDPlane.plane_count}'
    self.xform = UsdGeom.Xform.Define(stage, xform_path)
    self.prim = UsdGeom.Plane.Define(stage, plane_path)
    self.ref = stage.GetPrimAtPath(plane_path)

  def update_geom(self, new_geom):
    self.update_pos(new_geom.pos)
    self.update_size(new_geom.size)
    self.update_color(new_geom.rgba)

class USDSphere(USDGeom):
  """
  Stores information regarding a sphere geom in USD
  """

  sphere_count = 0

  def __init__(self, 
               geom=None,
               stage=None):
    super().__init__(geom, stage)
    self.type = 2
    USDSphere.sphere_count += 1
    xform_path = f'/Sphere_Xform_{USDSphere.sphere_count}'
    sphere_path = f'{xform_path}/Sphere_{USDSphere.sphere_count}'
    self.xform = UsdGeom.Xform.Define(stage, xform_path)
    self.prim = UsdGeom.Sphere.Define(stage, sphere_path)
    self.ref = stage.GetPrimAtPath(sphere_path)

  def update_geom(self, new_geom):
    self.update_pos(new_geom.pos)
    self.update_size(new_geom.size)
    self.update_color(new_geom.rgba)

class USDCube(USDGeom):
  """
  Stores information regarding a cube geom in USD
  """

  cube_count = 0

  def __init__(self, 
               geom=None,
               stage=None):
    super().__init__(geom, stage)
    self.type = 6
    USDCube.cube_count += 1
    xform_path = f'/Cube_Xform_{USDCube.cube_count}'
    cube_path = f'{xform_path}/Cube_{USDCube.cube_count}'
    self.xform = UsdGeom.Xform.Define(stage, xform_path)
    self.prim = UsdGeom.Cube.Define(stage, cube_path)
    self.ref = stage.GetPrimAtPath(cube_path)

  def update_geom(self, new_geom):
    self.update_pos(new_geom.pos)
    self.update_size(new_geom.size)
    self.update_color(new_geom.rgba)

def create_usd_geom(geom, stage):
  geom_type = geom.type
  if geom_type==USDGeomType.Plane.value:
    return USDPlane(geom, stage)
  elif geom_type==USDGeomType.Sphere.value:
    return USDSphere(geom, stage)
  elif geom_type==USDGeomType.Cube.value:
    return USDCube(geom, stage)
  
class USDLight(object):
  def __init__(self):
    pass

  def update_light(self, new_light):
    pass
  
