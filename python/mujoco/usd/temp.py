# Copyright 2024 DeepMind Technologies Limited
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
from abs import ABC
from typing import List, Optional, Tuple

import mujoco

# import mujoco.usd.utils as utils_component
# import mujoco.usd.shapes as shapes_component
import utils as utils_component
import shapes as shapes_component

import numpy as np

# TODO: b/288149332 - Remove once USD Python Binding works well with pytype.
# pytype: disable=module-attr
from open3d import open3d as o3d
from pxr import Gf
from pxr import Sdf
from pxr import Usd
from pxr import UsdGeom
from pxr import UsdLux
from pxr import UsdShade
from pxr import Vt

class USDMesh:

  def __init__(
      self,
      stage: Usd.Stage,
      model: mujoco.MjModel,
      geom: mujoco.MjvGeom,
      obj_name: str,
      dataid: int,
      rgba: np.ndarray = np.array([1, 1, 1, 1]),
      texture_file: Optional[str] = None,
  ):
    self.stage = stage
    self.model = model
    self.geom = geom
    self.obj_name = obj_name
    self.rgba = rgba
    self.dataid = dataid
    self.texture_file = texture_file

    xform_path = f"/World/Mesh_Xform_{obj_name}"
    mesh_path = f"{xform_path}/Mesh_{obj_name}"
    self.usd_xform = UsdGeom.Xform.Define(stage, xform_path)
    self.usd_mesh = UsdGeom.Mesh.Define(stage, mesh_path)
    self.usd_prim = stage.GetPrimAtPath(mesh_path)

    # setting mesh structure properties
    mesh_vert, mesh_face, mesh_facenum = self._get_mesh_geometry()
    self.usd_mesh.GetPointsAttr().Set(mesh_vert)
    self.usd_mesh.GetFaceVertexCountsAttr().Set(
        [3 for _ in range(mesh_facenum)]
    )
    self.usd_mesh.GetFaceVertexIndicesAttr().Set(mesh_face)

    # setting mesh uv properties
    mesh_texcoord, mesh_facetexcoord = self._get_uv_geometry()
    self.texcoords = UsdGeom.PrimvarsAPI(self.usd_mesh).CreatePrimvar(
        "UVMap", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
    )
    self.texcoords.Set(mesh_texcoord)
    self.texcoords.SetIndices(Vt.IntArray(mesh_facetexcoord.tolist()))

    self._attach_material()

    # defining ops required by update function
    self.transform_op = self.usd_xform.AddTransformOp()

    self.last_visible_frame = -2 # not an arbitary value, forces difference greater than 1 for visibility on 0th frame

  def get_facetexcoord_ranges(self, nmesh, arr):
    facetexcoords_ranges = [0]
    running_sum = 0
    for i in range(nmesh):
      running_sum += arr[i] * 3
      facetexcoords_ranges.append(running_sum)
    return facetexcoords_ranges

  def _get_uv_geometry(self):
    mesh_texcoord_adr_from = self.model.mesh_texcoordadr[self.dataid]
    mesh_texcoord_adr_to = (
        self.model.mesh_texcoordadr[self.dataid + 1]
        if self.dataid < self.model.nmesh - 1
        else len(self.model.mesh_texcoord)
    )
    mesh_texcoord = self.model.mesh_texcoord[
        mesh_texcoord_adr_from:mesh_texcoord_adr_to
    ]

    mesh_facetexcoord_ranges = self.get_facetexcoord_ranges(
        self.model.nmesh, self.model.mesh_facenum
    )

    mesh_facetexcoord = self.model.mesh_facetexcoord.flatten()
    mesh_facetexcoord = mesh_facetexcoord[
        mesh_facetexcoord_ranges[self.dataid] : mesh_facetexcoord_ranges[
            self.dataid + 1
        ]
    ]

    mesh_facetexcoord[mesh_facetexcoord == len(mesh_texcoord)] = 0

    return mesh_texcoord, mesh_facetexcoord

  def _get_mesh_geometry(self):
    mesh_vert_adr_from = self.model.mesh_vertadr[self.dataid]
    mesh_vert_adr_to = (
        self.model.mesh_vertadr[self.dataid + 1]
        if self.dataid < self.model.nmesh - 1
        else len(self.model.mesh_vert)
    )
    mesh_vert = self.model.mesh_vert[mesh_vert_adr_from:mesh_vert_adr_to]

    mesh_face_adr_from = self.model.mesh_faceadr[self.dataid]
    mesh_face_adr_to = (
        self.model.mesh_faceadr[self.dataid + 1]
        if self.dataid < self.model.nmesh - 1
        else len(self.model.mesh_face)
    )
    mesh_face = self.model.mesh_face[mesh_face_adr_from:mesh_face_adr_to]

    mesh_facenum = self.model.mesh_facenum[self.dataid]

    return mesh_vert, mesh_face, mesh_facenum

  def _attach_material(self):
    mtl_path = Sdf.Path(f"/World/_materials/Material_{self.obj_name}")
    mtl = UsdShade.Material.Define(self.stage, mtl_path)

    if self.texture_file:
      bsdf_shader = UsdShade.Shader.Define(
          self.stage, mtl_path.AppendPath("Principled_BSDF")
      )
      image_shader = UsdShade.Shader.Define(
          self.stage, mtl_path.AppendPath("Image_Texture")
      )
      uvmap_shader = UsdShade.Shader.Define(
          self.stage, mtl_path.AppendPath("uvmap")
      )

      # setting the bsdf shader attributes
      bsdf_shader.CreateIdAttr("UsdPreviewSurface")
      bsdf_shader.CreateInput(
          "diffuseColor", Sdf.ValueTypeNames.Color3f
      ).ConnectToSource(image_shader.ConnectableAPI(), "rgb")
      bsdf_shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(
          float(self.rgba[-1])
      )
      bsdf_shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(
          self.geom.shininess
      )
      bsdf_shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(
          1.0 - self.geom.shininess
      )

      mtl.CreateSurfaceOutput().ConnectToSource(
          bsdf_shader.ConnectableAPI(), "surface"
      )

      self.usd_mesh.GetPrim().ApplyAPI(UsdShade.MaterialBindingAPI)
      UsdShade.MaterialBindingAPI(self.usd_mesh).Bind(mtl)

      # setting the image texture attributes
      image_shader.CreateIdAttr("UsdUVTexture")
      image_shader.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
          self.texture_file
      )
      image_shader.CreateInput(
          "sourceColorSpace", Sdf.ValueTypeNames.Token
      ).Set("sRGB")
      image_shader.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
      image_shader.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
      image_shader.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
          uvmap_shader.ConnectableAPI(), "result"
      )
      image_shader.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

      # setting uvmap shader attributes
      uvmap_shader.CreateIdAttr("UsdPrimvarReader_float2")
      uvmap_shader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("UVMap")
      uvmap_shader.CreateOutput("results", Sdf.ValueTypeNames.Float2)
    else:
      bsdf_shader = UsdShade.Shader.Define(
          self.stage, mtl_path.AppendPath("Principled_BSDF")
      )

      # settings the bsdf shader attributes
      bsdf_shader.CreateIdAttr("UsdPreviewSurface")

      bsdf_shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
          tuple(self.rgba[0:3])
      )
      bsdf_shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(
          float(self.rgba[-1])
      )
      bsdf_shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(
          self.geom.shininess
      )
      bsdf_shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(
          1.0 - self.geom.shininess
      )

    mtl.CreateSurfaceOutput().ConnectToSource(
        bsdf_shader.ConnectableAPI(), "surface"
    )

    self.usd_mesh.GetPrim().ApplyAPI(UsdShade.MaterialBindingAPI)
    UsdShade.MaterialBindingAPI(self.usd_mesh).Bind(mtl)

  def update(self, pos: np.ndarray, mat: np.ndarray, visible: bool, frame: int):
    transformation_mat = utils_component.create_transform_matrix(
        rotation_matrix=mat, translation_vector=pos
    ).T
    self.transform_op.Set(Gf.Matrix4d(transformation_mat.tolist()), frame)
    
    if visible and frame - self.last_visible_frame > 1:
      # non consecutive visible frames
      self.update_visibility(False, max(0, self.last_visible_frame))
      self.update_visibility(True, frame)

    if visible:
      self.last_visible_frame = frame

  def update_visibility(self, visible: bool, frame: int):
    visibility_setting = "inherited" if visible else "invisible"
    self.usd_prim.GetAttribute("visibility").Set(visibility_setting, frame)


class USDPrimitiveMesh:

  def __init__(
      self,
      mesh_config: List[dict],
      stage: Usd.Stage,
      geom: mujoco.MjvGeom,
      obj_name: str,
      rgba: np.ndarray = np.array([1, 1, 1, 1]),
      texture_file: Optional[str] = None,
  ):
    self.mesh_config = mesh_config
    self.stage = stage
    self.geom = geom
    self.obj_name = obj_name
    self.rgba = rgba
    self.texture_file = texture_file

    self.usd_prim = Usd.Prim()
    self.usd_mesh = UsdGeom.Mesh()
    self.prim_mesh = None
    self.transform_op = Gf.Matrix4d(1.)

    self.prim_mesh = self.generate_primitive_mesh()

    xform_path = f"/World/{self.obj_name}_Xform"
    mesh_path = f"{xform_path}/{obj_name}"
    self.usd_xform = UsdGeom.Xform.Define(stage, xform_path)
    self.usd_mesh = UsdGeom.Mesh.Define(stage, mesh_path)
    self.usd_prim = stage.GetPrimAtPath(mesh_path)

    mesh_vert, mesh_face, mesh_facenum = self._get_mesh_geometry()
    self.usd_mesh.GetPointsAttr().Set(mesh_vert)
    self.usd_mesh.GetFaceVertexCountsAttr().Set(
        [3 for _ in range(mesh_facenum)]
    )
    self.usd_mesh.GetFaceVertexIndicesAttr().Set(mesh_face)

    # setting mesh uv properties
    mesh_texcoord, mesh_facetexcoord = self._get_uv_geometry()
    self.texcoords = UsdGeom.PrimvarsAPI(self.usd_mesh).CreatePrimvar(
        "UVMap", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
    )

    self.texcoords.Set(mesh_texcoord)
    self.texcoords.SetIndices(Vt.IntArray([i for i in range(mesh_facenum * 3)]))

    self._set_refinement_properties()

    # setting attributes for the shape
    self._attach_material()

    # defining ops required by update functions
    self.transform_op = self.usd_xform.AddTransformOp()
    self.scale_op = self.usd_xform.AddScaleOp()

    self.last_visible_frame = -2

  def generate_primitive_mesh(self):
    _, prim_mesh = shapes_component.mesh_generator(self.mesh_config)
    prim_mesh.translate(-prim_mesh.get_center())
    return prim_mesh

  def _set_refinement_properties(self):
    self.usd_prim.GetAttribute("subdivisionScheme").Set("none")

  def _get_uv_geometry(self):

    assert self.prim_mesh

    x_scale, y_scale = self.geom.texrepeat
    mesh_texcoord = np.array(self.prim_mesh.triangle_uvs)
    mesh_facetexcoord = np.asarray(self.prim_mesh.triangles)

    x_multiplier, y_multiplier = 1, 1
    if self.geom.texuniform:
      if "box" in self.mesh_config:
        x_multiplier, y_multiplier = self.mesh_config["box"]["width"], self.mesh_config["box"]["height"]
      elif "sphere" in self.mesh_config:
        x_multiplier, y_multiplier = self.mesh_config["sphere"]["radius"], self.mesh_config["sphere"]["radius"]
      
    mesh_texcoord[:, 0] *= x_scale * x_multiplier
    mesh_texcoord[:, 1] *= y_scale * y_multiplier

    return mesh_texcoord, mesh_facetexcoord.flatten()

  def _get_mesh_geometry(self):

    assert self.prim_mesh

    # get mesh geometry from the open3d mesh model
    mesh_vert = np.asarray(self.prim_mesh.vertices)
    mesh_face = np.asarray(self.prim_mesh.triangles)

    return mesh_vert, mesh_face, len(mesh_face)

  def _attach_material(self):
    mtl_path = Sdf.Path(f"/World/_materials/Material_{self.obj_name}")
    mtl = UsdShade.Material.Define(self.stage, mtl_path)
    if self.texture_file:
      bsdf_shader = UsdShade.Shader.Define(
          self.stage, mtl_path.AppendPath("Principled_BSDF")
      )
      image_shader = UsdShade.Shader.Define(
          self.stage, mtl_path.AppendPath("Image_Texture")
      )
      uvmap_shader = UsdShade.Shader.Define(
          self.stage, mtl_path.AppendPath("uvmap")
      )

      # setting the bsdf shader attributes
      bsdf_shader.CreateIdAttr("UsdPreviewSurface")
      bsdf_shader.CreateInput(
          "diffuseColor", Sdf.ValueTypeNames.Color3f
      ).ConnectToSource(image_shader.ConnectableAPI(), "rgb")
      bsdf_shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(
          float(self.rgba[-1])
      )
      bsdf_shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(
          self.geom.shininess
      )
      bsdf_shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(
          1.0 - self.geom.shininess
      )

      mtl.CreateSurfaceOutput().ConnectToSource(
          bsdf_shader.ConnectableAPI(), "surface"
      )

      self.usd_mesh.GetPrim().ApplyAPI(UsdShade.MaterialBindingAPI)
      UsdShade.MaterialBindingAPI(self.usd_mesh).Bind(mtl)

      # setting the image texture attributes
      image_shader.CreateIdAttr("UsdUVTexture")
      image_shader.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
          self.texture_file
      )
      image_shader.CreateInput(
          "sourceColorSpace", Sdf.ValueTypeNames.Token
      ).Set("sRGB")
      image_shader.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
      image_shader.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
      image_shader.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
          uvmap_shader.ConnectableAPI(), "result"
      )
      image_shader.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

      # setting uvmap shader attributes
      uvmap_shader.CreateIdAttr("UsdPrimvarReader_float2")
      uvmap_shader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("UVMap")
      uvmap_shader.CreateOutput("results", Sdf.ValueTypeNames.Float2)
    else:
      bsdf_shader = UsdShade.Shader.Define(
          self.stage, mtl_path.AppendPath("Principled_BSDF")
      )

      # settings the bsdf shader attributes
      bsdf_shader.CreateIdAttr("UsdPreviewSurface")
      bsdf_shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
          tuple(self.rgba[:3])
      )
      bsdf_shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(
          float(self.rgba[-1])
      )
      bsdf_shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(
          self.geom.shininess
      )
      bsdf_shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(
          1.0 - self.geom.shininess
      )

    mtl.CreateSurfaceOutput().ConnectToSource(
        bsdf_shader.ConnectableAPI(), "surface"
    )

    self.usd_mesh.GetPrim().ApplyAPI(UsdShade.MaterialBindingAPI)
    UsdShade.MaterialBindingAPI(self.usd_mesh).Bind(mtl)

  def update(self, pos: np.ndarray, mat: np.ndarray, visible: bool, frame: int, ):
    transformation_mat = utils_component.create_transform_matrix(
        rotation_matrix=mat, translation_vector=pos
    ).T
    self.transform_op.Set(Gf.Matrix4d(transformation_mat.tolist()), frame)

    if visible and frame - self.last_visible_frame > 1:
      # non consecutive visible frames
      self.update_visibility(False, max(0, self.last_visible_frame))
      self.update_visibility(True, frame)

    if visible:
      self.last_visible_frame = frame

  def update_visibility(self, visible: bool, frame: int):
    visibility_setting = "inherited" if visible else "invisible"
    self.usd_prim.GetAttribute("visibility").Set(visibility_setting, frame)

  def update_scale(self, scale: np.ndarray, frame: int):
    self.scale_op.Set(Gf.Vec3f(scale.tolist()), frame)

class USDTendon:
  
  def __init__(
      self
  ):
    pass

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
