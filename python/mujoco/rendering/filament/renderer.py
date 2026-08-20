# Copyright 2026 DeepMind Technologies Limited
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
"""Class for coordinating the rendering of scenes to render targets."""

import dataclasses
from typing import Any

import mujoco
from mujoco import _render_filament as mjrf
import numpy as np


class Renderer:
  """Coordinates the rendering of scenes to render targets.

  Internally, the Renderer manages the mjrf objects (i.e. Scene, Camera, and
  RenderTarget) that are needed to render a MuJoCo model. The Scene defines
  the entities from the model that are to be rendered. The Camera defines the
  viewpoint from which to render the model. And, finally, the RenderTarget
  defines the memory buffers into which the final rendered image will be
  written.

  Each object owned by the Renderer is associated with a unique name. This name
  is used to identify the object when making rendering calls.

  The Renderer does not specify the contents of the Scene; it is up to the user
  to populate the Scene with the entities that they want to render.

  The Renderer is not limited to rendering a single scene to a single target.
  You can configure the Renderer to manage multiple scenes from multiple
  cameras into multiple targets. Each such render operation is specified using
  the view() method.
  """

  def __init__(self, ctx: mjrf.Context):
    self._ctx = ctx
    self._scenes: dict[str, mjrf.Scene] = {}
    self._targets: dict[str, mjrf.RenderTarget] = {}
    self._views: dict[str, Renderer._View] = {}
    self._reads: dict[str, Renderer._Read] = {}
    self._ctx.set_clear_color([0.0, 0.0, 0.0])

  def scene(self, name: str) -> mjrf.Scene:
    """Returns the scene with the given name, creating it if needed."""
    if name not in self._scenes:
      self._scenes[name] = self._ctx.create_scene(mjrf.SceneParams())
    return self._scenes[name]

  def target(
      self,
      name: str,
      size: tuple[int, int],
      pixel_format=mjrf.PixelFormat.PIXEL_FORMAT_RGB8,
  ):
    """Defines a render target with the given name and size/dimensions."""
    match pixel_format:
      case mjrf.PixelFormat.PIXEL_FORMAT_R8:
        pixel_size = 1
      case mjrf.PixelFormat.PIXEL_FORMAT_RGB8:
        pixel_size = 3
      case mjrf.PixelFormat.PIXEL_FORMAT_RGBA8:
        pixel_size = 4
      case mjrf.PixelFormat.PIXEL_FORMAT_R32F:
        pixel_size = 4
      case mjrf.PixelFormat.PIXEL_FORMAT_DEPTH32F:
        pixel_size = 4
      case _:
        raise ValueError(f"Unsupported pixel format: {pixel_format}")
    num_bytes = size[0] * size[1] * pixel_size

    # Each target has a 1:1 association with a _Read object.
    if name not in self._targets:
      target = self._ctx.create_render_target(
          mjrf.RenderTargetConfig(
              color_format=pixel_format,
          )
      )
      self._targets[name] = target

      r = self._Read(
          size=size,
          format=pixel_format,
          request=mjrf.ReadPixelsRequest(),
      )
      r.request.target = target
      r.request.alloc(num_bytes)
      self._reads[name] = r
    else:
      r = self._reads[name]
      if r.format != pixel_format:
        raise ValueError(
            f"Target {name} already exists with a different pixel format."
        )

    target = self._targets[name]
    if size[0] != 0 and size[1] != 0:
      target.resize(size[0], size[1])
      r.size = size
      r.request.alloc(num_bytes)
    return target

  def view(
      self,
      name: str,
      *,
      scene: str,
      target: str,
      camera: mjrf.Camera | mujoco.MjvGLCamera = mjrf.Camera(),
      viewport: tuple[float, float, float, float] | None = None,
      draw_mode: mjrf.DrawMode = mjrf.DrawMode.DRAW_MODE_DEFAULT,
  ):
    """Associates a scene, camera, viewport, and target as a rendering operation."""
    self._views[name] = self._View(
        scene=scene,
        target=target,
        viewport=viewport or (0.0, 0.0, 1.0, 1.0),
        request=mjrf.RenderRequest(
            camera=self._to_mjrf_camera(camera),
            draw_mode=draw_mode,
        ),
    )

  def update_camera(
      self,
      name: str,
      camera: mjrf.Camera | mujoco.MjvGLCamera,
  ):
    """Updates the camera for the given view."""
    if name not in self._views:
      raise ValueError(f"View not found: {name}")
    self._views[name].request.camera = self._to_mjrf_camera(camera)

  def render(self):
    """Renders all active views."""
    requests: list[mjrf.RenderRequest] = []
    for view in self._views.values():
      read = self._reads.get(view.target, None)
      if read is None:
        raise ValueError(f"Target not found: {view.target}")

      scene = self._scenes.get(view.scene, None)
      if scene is None:
        raise ValueError(f"Scene not found: {view.scene}")

      view.request.scene = scene
      view.request.target = read.request.target
      view.request.viewport.left = int(view.viewport[0] * read.size[0])
      view.request.viewport.bottom = int(view.viewport[1] * read.size[1])
      view.request.viewport.width = int(view.viewport[2] * read.size[0])
      view.request.viewport.height = int(view.viewport[3] * read.size[1])
      requests.append(view.request)

    reads = [read.request for read in self._reads.values()]
    frame = self._ctx.render(requests, reads)
    self._ctx.wait_for_frame(frame)

  @dataclasses.dataclass
  class ImageResult:
    """A rendered image."""

    width: int
    height: int
    format: mjrf.PixelFormat
    pixels: bytes

    def __array_interface__(self) -> dict[str, Any]:
      return {
          "version": 3,
          "data": self.pixels,
          "shape": self.shape,
          "typestr": np.dtype(self.dtype).str,
      }

    def __array__(self, dtype=None, copy=None) -> np.ndarray:
      """Returns the image as a NumPy array."""
      req_dtype = np.dtype(dtype) if dtype is not None else self.dtype
      if req_dtype != self.dtype:
        raise ValueError(
            f"Unsupported dtype: {req_dtype}. Must be {self.dtype}."
        )

      return np.frombuffer(
          self.pixels,
          dtype=self.dtype,
      ).reshape(self.shape)

    @property
    def channels(self) -> int:
      """Returns the number of channels in the image."""
      match self.format:
        case mjrf.PixelFormat.PIXEL_FORMAT_R8:
          return 1
        case mjrf.PixelFormat.PIXEL_FORMAT_RGB8:
          return 3
        case mjrf.PixelFormat.PIXEL_FORMAT_RGBA8:
          return 4
        case mjrf.PixelFormat.PIXEL_FORMAT_R32F:
          return 1
        case mjrf.PixelFormat.PIXEL_FORMAT_DEPTH32F:
          return 1
        case _:
          raise ValueError(f"Unsupported pixel format: {self.format}")

    @property
    def dtype(self) -> np.dtype:
      """Returns the NumPy dtype of the image."""
      match self.format:
        case mjrf.PixelFormat.PIXEL_FORMAT_R8:
          return np.dtype(np.uint8)
        case mjrf.PixelFormat.PIXEL_FORMAT_RGB8:
          return np.dtype(np.uint8)
        case mjrf.PixelFormat.PIXEL_FORMAT_RGBA8:
          return np.dtype(np.uint8)
        case mjrf.PixelFormat.PIXEL_FORMAT_R32F:
          return np.dtype(np.float32)
        case mjrf.PixelFormat.PIXEL_FORMAT_DEPTH32F:
          return np.dtype(np.float32)
        case _:
          raise ValueError(f"Unsupported pixel format: {self.format}")

    @property
    def shape(self) -> tuple[int, int, int]:
      """Returns the shape of the image."""
      return (self.height, self.width, self.channels)

  def get_image(self, name: str) -> ImageResult:
    """Returns the most recently rendered pixels for the given render target."""
    read = self._reads[name]
    pixels = read.request.buffer()
    return self.ImageResult(
        width=read.size[0],
        height=read.size[1],
        format=read.format,
        pixels=pixels,
    )

  def _to_mjrf_camera(
      self,
      camera: mjrf.Camera | mujoco.MjvGLCamera,
  ) -> mjrf.Camera:
    """Converts a MjvGLCamera to an mjrf.Camera."""
    if isinstance(camera, mujoco.MjvGLCamera):
      # In C, mjrCamera is a type alias for mjvGLCamera. However, they are
      # bound as distinct types in Python, so we convert from MjvGLCamera to
      # mjrf.Camera here.
      camera = mjrf.Camera(
          pos=camera.pos,
          forward=camera.forward,
          up=camera.up,
          frustum_bottom=camera.frustum_bottom,
          frustum_top=camera.frustum_top,
          frustum_near=camera.frustum_near,
          frustum_far=camera.frustum_far,
      )
    return camera

  @dataclasses.dataclass
  class _View:
    """Internal wrapper around mjrf.RenderRequest."""
    scene: str
    target: str
    viewport: tuple[float, float, float, float]
    request: mjrf.RenderRequest

  @dataclasses.dataclass
  class _Read:
    """Internal wrapper around mjrf.ReadPixelsRequest."""
    size: tuple[int, int]
    format: mjrf.PixelFormat
    request: mjrf.ReadPixelsRequest
