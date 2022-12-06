# Copyright 2022 DeepMind Technologies Limited
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
"""Defines a renderer class for the MuJoCo Python native bindings."""

from typing import Optional, Union

from mujoco import _enums
from mujoco import _functions
from mujoco import _render
from mujoco import _structs
from mujoco import gl_context
import numpy as np


class Renderer:
  """Renders MuJoCo scenes."""

  def __init__(
      self,
      model: _structs.MjModel,
      height: int = 240,
      width: int = 320,
      max_geom: int = 10000
  ) -> None:
    """Initializes a new `Renderer`.

    Args:
      model: an MjModel instance.
      height: image height in pixels.
      width: image width in pixels.
      max_geom: Optional integer specifying the maximum number of geoms that can
        be rendered in the same scene. If None this will be chosen automatically
        based on the estimated maximum number of renderable geoms in the model.
    Raises:
      ValueError: If `camera_id` is outside the valid range, or if `width` or
        `height` exceed the dimensions of MuJoCo's offscreen framebuffer.
    """
    buffer_width = model.vis.global_.offwidth
    buffer_height = model.vis.global_.offheight
    if width > buffer_width:
      raise ValueError(f"""
Image width {width} > framebuffer width {buffer_width}. Either reduce the image
width or specify a larger offscreen framebuffer in the model XML using the
clause:
<visual>
  <global offwidth="my_width"/>
</visual>""".lstrip())

    if height > buffer_height:
      raise ValueError(f"""
Image height {height} > framebuffer height {buffer_height}. Either reduce the
image height or specify a larger offscreen framebuffer in the model XML using
the clause:
<visual>
  <global offheight="my_height"/>
</visual>""".lstrip())

    self._width = width
    self._height = height
    self._model = model

    self._scene = _structs.MjvScene(model=model, maxgeom=max_geom)
    self._scene_option = _structs.MjvOption()

    self._rect = _render.MjrRect(0, 0, self._width, self._height)

    # Internal buffers.
    self._rgb_buffer = np.empty((self._height, self._width, 3), dtype=np.uint8)
    self._depth_buffer = np.empty((self._height, self._width), dtype=np.float32)

    # Create render contexts.
    self._gl_context = gl_context.GLContext(width, height)
    self._gl_context.make_current()
    self._mjr_context = _render.MjrContext(
        model, _enums.mjtFontScale.mjFONTSCALE_150
    )
    _render.mjr_setBuffer(
        _enums.mjtFramebuffer.mjFB_OFFSCREEN, self._mjr_context
    )

    # Default render flags.
    self._depth_rendering = False
    self._segmentation_rendering = False

  @property
  def model(self):
    return self._model

  @property
  def scene(self) -> _structs.MjvScene:
    return self._scene

  @property
  def height(self):
    return self._height

  @property
  def width(self):
    return self._width

  def enable_depth_rendering(self):
    self._segmentation_rendering = False
    self._depth_rendering = True

  def disable_depth_rendering(self):
    self._depth_rendering = False

  def enable_segmentation_rendering(self):
    self._segmentation_rendering = True
    self._depth_rendering = False

  def disable_segmentation_rendering(self):
    self._segmentation_rendering = False

  def render(self) -> np.ndarray:
    """Renders the scene as a numpy array of pixel values.

    Returns:
      A numpy array of pixels with dimensions (H, W, 3). The array will be
      mutated by future calls to `render`.
    """
    original_flags = self._scene.flags.copy()

    if self._segmentation_rendering:
      self._scene.flags[_enums.mjtRndFlag.mjRND_SEGMENT] = True
      self._scene.flags[_enums.mjtRndFlag.mjRND_IDCOLOR] = True

    self._gl_context.make_current()

    # Render scene and read contents of RGB and depth buffers.
    _render.mjr_render(self._rect, self._scene, self._mjr_context)
    _render.mjr_readPixels(self._rgb_buffer, self._depth_buffer, self._rect,
                           self._mjr_context)

    if self._depth_rendering:
      # Get the distances to the near and far clipping planes.
      extent = self._model.stat.extent
      near = self._model.vis.map.znear * extent
      far = self._model.vis.map.zfar * extent

      # Convert from [0 1] to depth in units of length, see links below:
      # http://stackoverflow.com/a/6657284/1461210
      # https://www.khronos.org/opengl/wiki/Depth_Buffer_Precision
      pixels = near / (1 - self._depth_buffer * (1 - near / far))

    elif self._segmentation_rendering:
      # Convert 3-channel uint8 to 1-channel uint32.
      image3 = self._rgb_buffer.astype(np.uint32)
      segimage = (image3[:, :, 0] +
                  image3[:, :, 1] * (2**8) +
                  image3[:, :, 2] * (2**16))
      # Remap segid to 2-channel (object ID, object type) pair.
      # Seg ID 0 is background -- will be remapped to (-1, -1).
      ngeoms = self._scene.ngeom
      segid2output = np.full((ngeoms + 1, 2), fill_value=-1,
                             dtype=np.int32)  # Seg id cannot be > ngeom + 1.
      visible_geoms = [g for g in self._scene.geoms[:ngeoms] if g.segid != -1]
      visible_segids = np.array([g.segid + 1 for g in visible_geoms], np.int32)
      visible_objid = np.array([g.objid for g in visible_geoms], np.int32)
      visible_objtype = np.array([g.objtype for g in visible_geoms], np.int32)
      segid2output[visible_segids, 0] = visible_objid
      segid2output[visible_segids, 1] = visible_objtype
      pixels = segid2output[segimage]

      # Reset scene flags.
      np.copyto(self._scene.flags, original_flags)
    else:
      pixels = self._rgb_buffer
    return np.flipud(pixels)

  def update_scene(
      self,
      data: _structs.MjData,
      camera: Union[int, str, _structs.MjvCamera] = -1,
      scene_option: Optional[_structs.MjvOption] = None
    ):
    """Updates geometry used for rendering.

    Args:
      data: An instance of `MjData`.
      camera: An instance of `MjvCamera`, a string or an integer
      scene_option: A custom `MjvOption` instance to use to render
        the scene instead of the default.
    """
    if not isinstance(camera, _structs.MjvCamera):
      camera_id = camera
      if isinstance(camera_id, str):
        camera_id = _functions.mj_name2id(self._model,
                                          _enums.mjtObj.mjOBJ_CAMERA, camera_id)
      if camera_id < -1:
        raise ValueError('camera_id cannot be smaller than -1.')
      if camera_id >= self._model.ncam:
        raise ValueError(
            f'model has {self._model.ncam} fixed cameras. '
            f'camera_id={camera_id} is invalid.'
        )

      # Render camera.
      camera = _structs.MjvCamera()
      camera.fixedcamid = camera_id

      # Defaults to mjCAMERA_FREE, otherwise mjCAMERA_FIXED refers to a
      # camera explicitly defined in the model.
      if camera_id == -1:
        camera.type = _enums.mjtCamera.mjCAMERA_FREE
        _functions.mjv_defaultFreeCamera(self._model, camera)
      else:
        camera.type = _enums.mjtCamera.mjCAMERA_FIXED

    scene_option = scene_option or self._scene_option
    _functions.mjv_updateScene(
        self._model,
        data,
        scene_option,
        None,
        camera, _enums.mjtCatBit.mjCAT_ALL,
        self._scene,
    )
