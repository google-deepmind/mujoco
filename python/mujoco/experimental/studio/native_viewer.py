# Copyright 2026 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Simulation-agnostic native viewer for MuJoCo models.

See the documentation for studio_app.py for more details on the architecture
separating the viewer and simulation. See the sample/ folder for examples of how
to use these classes.
"""

from typing import Any

import mujoco
from mujoco.experimental.studio import endpoints
from mujoco.experimental.studio import native_viewer_cc as _viewer
from mujoco.experimental.studio import ux
from mujoco.experimental.studio import viewer_protocol

from mujoco.experimental.dear_imgui import dear_imgui as imgui


class NativeViewer(viewer_protocol.Viewer):
  """Simulation-agnostic native viewer for MuJoCo models."""

  def __init__(
      self,
      config: viewer_protocol.ViewerConfig,
      endpoint: endpoints.ViewerEndpoint,
      *,
      model: mujoco.MjModel | None = None,
      model_path: str = '',
      handlers: list[Any] | None = None,
      camera: mujoco.MjvCamera | None = None,
      vis_options: mujoco.MjvOption | None = None,
      perturb: mujoco.MjvPerturb | None = None,
      render_flags: ux.RenderFlags | None = None,
      extra_geoms: list[mujoco.MjvGeom] | None = None,
  ) -> None:
    """Initializes the NativeViewer.

    Args:
      config: Viewer window configuration.
      endpoint: The viewer endpoint for communication with the sim side.
      model: Optional initial MjModel. Forwarded to the base Viewer.
      model_path: Optional path to the model file.
      handlers: Optional list of handler instances.
      camera: Camera parameters. Internal object is created if None.
      vis_options: Visualization options. Internal object is created if None.
      perturb: Perturbation parameters. Internal object is created if None.
      render_flags: Render flags. Internal object is created if None.
      extra_geoms: List of extra geoms. Internal list is created if None.
    """
    super().__init__(
        config,
        endpoint,
        model=model,
        model_path=model_path,
        handlers=handlers,
        camera=camera,
        vis_options=vis_options,
        perturb=perturb,
        render_flags=render_flags,
        extra_geoms=extra_geoms,
    )

    # Create the renderer.
    self._viewer = _viewer.Viewer(
        config.title, config.width, config.height, config.gfx or ''
    )

    # Track the python object id of the model currently loaded in the renderer
    # so we can detect when the model changes and re-initialize.
    self._renderer_model_id = id(None)

    ctx = self._viewer.GetImGuiContext()
    imgui.SetCurrentContext(ctx)
    ux.set_imgui_context(ctx)

    # Dispatch lifecycle event so handlers can cache the viewer reference.
    self.dispatch(viewer_protocol.ViewerInitEvent(viewer=self))

  def _sync_renderer(self, model: mujoco.MjModel) -> None:
    """Re-initializes the renderer if the model object has changed."""
    if id(model) != self._renderer_model_id:
      self._viewer.InitRenderer(model)
      self._renderer_model_id = id(model)

  def is_running(self) -> bool:
    """Poll for a new frame; returns ``False`` when the window is closed."""
    if super().is_running() and not self._viewer.NewFrame():
      self.close()
    return super().is_running()

  def sync(self) -> None:
    """Render the scene and present it to the window."""
    self._sync_renderer(self.model)
    self._viewer.Present(
        self.model,
        self.data,
        self.perturb,
        self.camera,
        self.vis_options,
        self.render_flags.flags,
        self.extra_geoms,
    )

  # TODO(matijak): Remove stop() and rename callers to close().
  def stop(self) -> None:
    """Stop the viewer."""
    self.close()

  def get_drop_file(self) -> str:
    """Returns the path of the file dropped into the window, or empty string."""
    return self._viewer.GetDropFile()

  def upload_image(
      self, tex_id: int, img: str | bytes, width: int, height: int, bpp: int
  ) -> int:
    """Uploads an image to the backend for GUI rendering.

    The ID can be used in subsequent calls to update the texture data. An empty
    `img` argument will free the texture if it exists. A `tex_id` of 0 will
    create a new texture.

    Args:
      tex_id: The texture ID.
      img: The image data as string or bytes.
      width: Width of the image.
      height: Height of the image.
      bpp: Bytes per pixel.

    Returns:
      The texture ID.
    """
    return self._viewer.UploadImage(tex_id, img, width, height, bpp)

  def render_to_texture(
      self,
      model: mujoco.MjModel,
      data: mujoco.MjData,
      tex_id: int,
      width: int,
      height: int,
  ) -> int:
    """Renders the scene to a texture.

    This function renders the scene from the current camera view into a texture.
    It handles buffer allocation internally.

    Args:
      model: The MuJoCo model provided by the simulation.
      data: The MuJoCo data provided by the simulation.
      tex_id: The texture ID to render into (0 to create a new one).
      width: Width of the texture.
      height: Height of the texture.

    Returns:
      The texture ID.
    """
    self._sync_renderer(model)
    return self._viewer.RenderToTexture(
        model,
        data,
        self.camera,
        width,
        height,
        tex_id,
    )
