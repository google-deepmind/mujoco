from enum import Enum

import mujoco
from pxr import Usd, UsdGeom, UsdLux, Vt, Gf
from scipy.spatial.transform import Rotation as R

def create_usd_geom_primitive(geom, stage):
  geom_type = geom.type
  if geom_type==USDGeomType.Plane.value:
    return USDPlane(geom, stage)
  elif geom_type==USDGeomType.Sphere.value:
    return USDSphere(geom, stage)
  elif geom_type==USDGeomType.Cube.value:
    return USDCube(geom, stage)

class USDGeomType(Enum):
  """
  Represents different types of geoms we can add to USD
  The values match those found by the enum presented here:
  https://mujoco.readthedocs.io/en/latest/APIreference/APItypes.html#mjtgeom
  """
  Plane = 0
  Sphere = 2
  Cube = 6
  Mesh = 7

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
    self.update_pos(new_geom.pos)
    self.update_rotation(new_geom.mat)
    self.update_size(new_geom.size)
    self.update_color(new_geom.rgba)
  
  def update_pos(self, new_pos):
    pos = tuple([float(x) for x in new_pos])
    self.xform.AddTranslateOp().Set(pos)

  def update_rotation(self, new_mat):
    r = R.from_matrix(new_mat)
    euler_rotation = r.as_euler('xyz', degrees=True)
    rotation = Gf.Vec3f(float(euler_rotation[0]), float(euler_rotation[1]), float(euler_rotation[2]))
    self.xform.AddRotateXYZOp().Set(rotation)

  # TODO: check to make sure scale and size are the same thing
  def update_size(self, new_size):
    size = tuple([float(x) for x in new_size])
    self.xform.AddScaleOp().Set(value=size)

  def update_color(self, new_color):
    # new_color is the rgba (we extract first three)
    rgba = [(float(x) for x in new_color[:3])]
    color = self.prim.GetDisplayColorAttr().Set(rgba)

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

  def update_size(self, new_size):
    self.prim.GetAxisAttr().Set("Z")
    self.prim.GetWidthAttr().Set(float(new_size[0]))
    self.prim.GetLengthAttr().Set(float(new_size[1]))

class USDSphere(USDGeom):
  """
  Stores information regarding a sphere geom in USD
  """

  sphere_count = 0

  def __init__(self, 
               geom,
               stage):
    super().__init__(geom, stage)
    self.type = 2
    USDSphere.sphere_count += 1
    xform_path = f'/Sphere_Xform_{USDSphere.sphere_count}'
    sphere_path = f'{xform_path}/Sphere_{USDSphere.sphere_count}'
    self.xform = UsdGeom.Xform.Define(stage, xform_path)
    self.prim = UsdGeom.Sphere.Define(stage, sphere_path)
    self.ref = stage.GetPrimAtPath(sphere_path)

class USDCube(USDGeom):
  """
  Stores information regarding a cube geom in USD
  """

  cube_count = 0

  def __init__(self, 
               geom,
               stage):
    super().__init__(geom, stage)
    self.type = 6
    USDCube.cube_count += 1
    xform_path = f'/Cube_Xform_{USDCube.cube_count}'
    cube_path = f'{xform_path}/Cube_{USDCube.cube_count}'
    self.xform = UsdGeom.Xform.Define(stage, xform_path)
    self.prim = UsdGeom.Cube.Define(stage, cube_path)
    self.ref = stage.GetPrimAtPath(cube_path)

class USDMesh(USDGeom):
  """
  Stores information regarding a mesh geom in USD
  """

  mesh_count = 0

  def __init__(self,
               mesh_idx,
               geom,
               stage,
               model,
               mesh_vertex_ranges,
               mesh_face_ranges):
    super().__init__(geom, stage)
    self.type = 7
    USDMesh.mesh_count += 1
    xform_path = f'/Mesh_Xform_{USDMesh.mesh_count}'
    mesh_path= f'{xform_path}/Mesh_{USDMesh.mesh_count}'
    self.xform = UsdGeom.Xform.Define(stage, xform_path)
    self.prim = UsdGeom.Mesh.Define(stage, mesh_path)
    self.ref = stage.GetPrimAtPath(mesh_path)
    
    self.vertices = model.mesh_vert[mesh_vertex_ranges[mesh_idx]:mesh_vertex_ranges[mesh_idx+1]]
    self.prim.GetPointsAttr().Set(self.vertices)

    self.prim.GetFaceVertexCountsAttr().Set([3 for _ in range(model.mesh_facenum[mesh_idx])])

    self.faces = model.mesh_face[mesh_face_ranges[mesh_idx]:mesh_face_ranges[mesh_idx+1]]
    self.prim.GetFaceVertexIndicesAttr().Set(self.faces)

  def update_geom(self, new_geom):
    self.update_pos(new_geom.pos)
    self.update_rotation(new_geom.mat)

    # TODO: remove this, temporary
    self.xform.AddScaleOp().Set(value=(10.0, 10.0, 10.0))
  
class USDLight(object):
  """
  Class for the created lights
  """

  light_count = 0

  def __init__(self,
               stage):
    self.stage = stage
    USDLight.light_count += 1
    xform_path = f'/Light_Xform_{USDLight.light_count}'
    light_path = f'{xform_path}/Light_{USDLight.light_count}'
    self.xform = UsdGeom.Xform.Define(stage, xform_path)
    self.prim = UsdLux.SphereLight.Define(stage, light_path)
    self.ref = stage.GetPrimAtPath(light_path)

  def update_light(self, new_light):
    pos = tuple([float(x) for x in new_light.pos])
    self.xform.AddTranslateOp().Set(pos)

    # TODO attributes:
    #   - direction
    #   - intensity
    #   - exposure
    #   - radius
    #   - specular

    self.prim.GetIntensityAttr().Set(5000);





    