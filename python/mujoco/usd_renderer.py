import os
import shutil
from termcolor import colored
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
  Renderer class that creates USD representations for mujoco scenes
  """
  def __init__(self,
               model,
               height=480,
               width=480,
               root_dir_name="usdpkg",
               root_dir_path=None,
               verbose=True):
    self.model = model
    self.root_dir_name = root_dir_name
    self.root_dir_path = root_dir_path
    self.verbose = verbose
    self.data = None
    self.renderer = mujoco.Renderer(model, height, width)
    self.reload_scene_info = True
    self.frame_count = 0

    self.create_output_directories()
      
    self.stage = Usd.Stage.CreateInMemory()

    UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)

    geom_groups = [0,1,0,0,0,0] # Setting default geom groups for now

    self.scene_option = _structs.MjvOption()
    self.scene_option.geomgroup = geom_groups

  @property
  def usd(self):
    return self.stage.GetRootLayer().ExportToString()
  
  @property
  def scene(self):
    return self.renderer.scene
  
  def create_output_directories(self):
    if not self.root_dir_path:
      self.root_dir_path = os.getcwd()

    self.output_dir = os.path.join(self.root_dir_path, self.root_dir_name)
    if not os.path.exists(self.output_dir):
      os.makedirs(self.output_dir)

    self.scenes_dir = os.path.join(self.output_dir, "scenes")
    if not os.path.exists(self.scenes_dir):
      os.makedirs(self.scenes_dir)
    
    self.assets_dir = os.path.join(self.output_dir, "assets")
    if not os.path.exists(self.assets_dir):
      os.makedirs(self.assets_dir)

    if self.verbose:
      output_dir_msg = colored(f"Writing files to {self.output_dir}", "green")
      print(output_dir_msg)

  def save_scene(self):
    output_file_path = os.path.join(self.scenes_dir, f'frame_{self.frame_count}_.usd')
    with open(output_file_path, "w") as f:
      f.write(self.usd)
    self.frame_count += 1 

  def update_geom_groups(self, geom_groups):
    self.scene_option.geomgroup = geom_groups
    self.reload_scene_info = True
    self.update_scene(self.data)

  def update_scene(self, data):
    self.renderer.update_scene(data, scene_option=self.scene_option)
    self.data = data

    if self.reload_scene_info:
      # loads the initial geoms, lights, and camera information 
      # from the scene
      self._load()
      self.reload_scene_info = False
    
    self._update()

  def _load(self):
    """
    Loads and initializes the necessary objects to render the scene
    """

    # Create and loads the texture files to the assets directory
    # TODO: remove code once added internally to mujoco
    data_adr = 0
    texture_files = []
    for texid in range(self.model.ntex):
      height = self.model.tex_height[texid]
      width = self.model.tex_width[texid]
      pixels = 3*height*width
      rgb = self.model.tex_rgb[data_adr:data_adr+pixels]
      img = rgb.reshape(height, width, 3)
      texture_file_name = f"texture_{texid}.png"
      file_path = os.path.join(self.assets_dir, texture_file_name)
      img = im.fromarray(img)
      img = ImageOps.flip(img)
      img.save(file_path)

      relative_path = os.path.relpath(self.assets_dir, self.scenes_dir)
      img_path = os.path.join(relative_path, texture_file_name)

      texture_files.append(img_path)
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
      if self.usd_geoms[i]: # TODO: remove this once all primitives are added
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

  def compress(self):
    """
    Compresses the output directory to a zip file for easy transfer
    """
    if self.verbose:
      output_dir_msg = colored(f"Compressing files at {self.output_dir} and saving at {self.output_dir}", "green")
      print(output_dir_msg)
    shutil.make_archive(base_name=self.output_dir, 
                        format='zip', 
                        base_dir=self.root_dir_name)

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