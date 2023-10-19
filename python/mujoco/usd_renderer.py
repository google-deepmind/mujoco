import os
import mujoco
import mujoco.viewer as viewer
from mujoco.usd_component import *
from mujoco.usd_utilities import *
from pxr import Usd, UsdGeom

from mujoco import _structs

from PIL import Image as im
from PIL import ImageOps

class USDRenderer(object):
  """
  Renderer class the creates USD representations for mujoco scenes
  """
  def __init__(self,
               model,
               height=480,
               width=480,
               geom_groups=[1,1,1,1,1,1],
               output_frame_dir="frame_tmp"):
    self.model = model
    self.data = None
    self.renderer = mujoco.Renderer(model, height, width)
    self.loaded_scene_info = False
    self.frame_count = 0
    self.output_frame_dir = output_frame_dir
    if not os.path.exists(self.output_frame_dir):
      os.mkdir(self.output_frame_dir)

    # Directory to store all image assets used by the usd scene
    if not os.path.exists("image_assets"):
      os.mkdir("image_assets")
      
    self.stage = Usd.Stage.CreateInMemory()

    UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)

    # TODO: maybe change where we initialize this?
    self.scene_option = _structs.MjvOption()
    self.scene_option.geomgroup = geom_groups

  @property
  def usd(self):
    return self.stage.GetRootLayer().ExportToString()
  
  @property
  def scene(self):
    return self.renderer.scene
  
  def save_scene(self):
    output_file_path = os.path.join(self.output_frame_dir, f'frame_{self.frame_count}.usd')
    with open(output_file_path, "w") as f:
      f.write(self.usd)
    self.frame_count += 1 

  def update_scene(self, data):
    self.renderer.update_scene(data, scene_option=self.scene_option)
    self.data = data

    if not self.loaded_scene_info:
      # loads the initial geoms, lights, and camera information 
      # from the scene
      self._load()
      self.loaded_scene_info = True
    
    self._update()

  def _load(self):
    """
    Loads and initializes the necessary objects to render the scene
    """

    # create and load the texture files
    # iterate through all the textures and build list of tex_rgb ranges
    # TODO: remove once added to mujoco
    data_adr = 0
    texture_files = []
    for texid in range(self.model.ntex):
      height = self.model.tex_height[texid]
      width = self.model.tex_width[texid]
      pixels = 3*height*width
      rgb = self.model.tex_rgb[data_adr:data_adr+pixels]
      img = rgb.reshape(height, width, 3)
      file_name = f'image_assets/{texid}.png'
      img = im.fromarray(img)
      img = ImageOps.flip(img)
      img.save(file_name)
      texture_file = os.path.abspath(file_name)
      texture_files.append(texture_file)
      data_adr += pixels

    # initializes an array to store all the geoms in the scene
    # populates with "empty" USDGeom objects
    self.usd_geoms = []
    geoms = self.scene.geoms
    self.ngeom = self.scene.ngeom
    for i in range(self.ngeom):
      geom = geoms[i]
      if geom.texid == -1:
        texture_file = None
      else:
        texture_file = texture_files[geom.texid]

      if geom.type == USDGeomType.Mesh.value:
        self.usd_geoms.append(USDMesh(self.model.geom_dataid[geom.objid],
                                        geom, 
                                        self.stage, 
                                        self.model,
                                        texture_file))
      else:
        self.usd_geoms.append(create_usd_geom_primitive(geom, 
                                                        self.stage,
                                                        texture_file))

    # initializes an array to store all the lights in the scene
    # populates with "empty" USDLight objects
    self.usd_lights = []
    lights = self.scene.lights
    self.nlight = self.scene.nlight
    for i in range(self.nlight):
      self.usd_lights.append(USDLight(self.stage))

    # initializes an array to store all the cameras in the scene
    # populates with "empty" USDCamera objects
    self.usd_cameras = []
    ncam = self.model.ncam
    for i in range(ncam):
      self.usd_cameras.append(USDCamera(self.stage))

  def _update(self):
    self._update_geoms()
    self._update_lights()
    self._update_cameras()

  def _update_geoms(self):
    """
    Updates the geoms to match the current scene
    """
    geoms = self.scene.geoms
    for i in range(self.ngeom):
      if self.usd_geoms[i] != None: # TODO: remove this once all primitives are added
        self.usd_geoms[i].update_geom(geoms[i])

  def _update_lights(self):
    """
    Updates the lights to match the current scene
    """
    lights = self.scene.lights
    nlight = self.scene.nlight
    for i in range(nlight):
      self.usd_lights[i].update_light(lights[i])
      
  def _update_cameras(self):
    """
    Updates the camera to match the current scene
    """
    ncam = self.model.ncam
    for i in range(ncam):
      self.usd_cameras[i].update_camera(self.model.cam_pos[i], self.model.cam_quat[i])

  def start_viewer(self):
    if self.data:
      viewer.launch(self.model)

  def render(self):
    # should render the usd file given a particular renderer that
    # works with USD files
    # TODO: determine if this is valid functionality
    pass

  # TODO: remove later, this is only for debugging purposes
  def print_geom_information(self):
    for i in range(self.ngeom):
      print(self.usd_geoms[i])