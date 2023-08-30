
import mujoco
import mujoco.viewer as viewer
from mujoco.usd_component import *
from pxr import Usd, UsdGeom

class USDRenderer(object):
  """
  Renderer class the creates USD representations for mujoco scenes
  """
  def __init__(self,
               model,
               height=480,
               width=480):
    self.model = model
    self.data = None
    self.renderer = mujoco.Renderer(model, height, width)

    self.loaded_scene_info = False

    self.stage = Usd.Stage.CreateNew('usd_stage.usda')

  @property
  def usd(self):
    return self.stage.GetRootLayer().ExportToString()
  
  @property
  def scene(self):
    return self.renderer.scene
  
  def save_scene(self):
    self.stage.GetRootLayer().Save()

  def update_scene(self, data):
    self.renderer.update_scene(data)
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
    # initializes an array to store all the geoms in the scene
    # populates with "empty" USDGeom objects
    self.usd_geoms = []
    geoms = self.scene.geoms
    self.ngeom = self.scene.ngeom
    for i in range(self.ngeom):
      self.usd_geoms.append(create_usd_geom(geoms[i], self.stage))

    # initializes an array to store all the lights in the scene
    # populates with "empty" USDLight objects
    self.usd_lights = []
    lights = self.scene.lights
    self.nlight = self.scene.nlight
    for i in range(self.nlight):
      self.usd_lights.append(USDLight())

  def _update(self):
    self._update_geoms()
    self._update_lights()
    self._update_camera()

  def _update_geoms(self):
    """
    Updates the geoms to match the current scene
    """
    geoms = self.scene.geoms
    for i in range(self.ngeom):
      self.usd_geoms[i].update_geom(geoms[i])

  def _update_lights(self):
    """
    Updates the lights to match the current scene
    """
    lights = self.scene.lights
    nlight = self.scene.nlight
    for i in range(nlight):
      print(self.usd_lights[i])
      
  def _update_camera(self):
    pass

  def start_viewer(self):
    if self.data:
      viewer.launch(self.model, self.data)

  def render(self):
    # should render the usd file given a particular renderer that
    # works with USD files?
    # TODO: determine if this is valid functionality
    pass

  # TODO: remove later, this is only for debuggin purposes
  def print_geom_information(self):
    for i in range(self.ngeom):
      print(self.usd_geoms[i])