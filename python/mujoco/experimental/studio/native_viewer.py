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

This class is simulation-agnostic and as such it does not own the model or data.

See the documentation for studio_app.py for more details on the architecture
separating the viewer and simulation. See the sample/ folder for examples of how
to use these classes.
"""

import mujoco

from mujoco.experimental.studio import native_viewer_cc as _viewer
from mujoco.experimental.studio import ux


class NativeViewer:
  """Simulation-agnostic native viewer for MuJoCo models."""

  def __init__(
      self,
      model: mujoco.MjModel,
      camera: mujoco.MjvCamera | None = None,
      vis_options: mujoco.MjvOption | None = None,
      perturb: mujoco.MjvPerturb | None = None,
      render_flags: ux.RenderFlags | None = None,
      title: str = '',
      width: int = 1200,
      height: int = 800,
      gfx: str = '',
  ) -> None:
    """Initializes the NativeViewer.

    The viewer creates and modifies its own camera, perturbation, and
    visualization option objects unless they are provided.

    Args:
      model: The MuJoCo model, used to initialize the renderer.
      camera: Camera parameters. Internal object is created if None.
      vis_options: Visualization options. Internal object is created if None.
      perturb: Perturbation parameters. Internal object is created if None.
      render_flags: Render flags. Internal object is created if None.
      title: Title of the viewer window.
      width: Initial width of the viewer window.
      height: Initial height of the viewer window.
      gfx: Graphics mode.
    """
    self.camera = camera or mujoco.MjvCamera()
    self.perturb = perturb or mujoco.MjvPerturb()
    self.vis_options = vis_options or mujoco.MjvOption()
    self._viewer = _viewer.Viewer(title, width, height, gfx)
    self._viewer.InitRenderer(model)
    # This class does not own the model but we need to know if the model being
    # rendered has changed, so we store the unique python object id here so we
    # can use it to detect model changes.
    self._renderer_model_id = id(model)
    self._is_running = True
    if render_flags is not None:
      self.render_flags = render_flags
    else:
      self.render_flags = ux.RenderFlags()
      # Initted to match mujoco/src/engine/engine_vis_init.c
      self.render_flags.flags = [1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1]

  def _sync_renderer(self, model: mujoco.MjModel) -> None:
    """Re-initializes the renderer if the model object has changed."""
    if id(model) != self._renderer_model_id:
      self._viewer.InitRenderer(model)
      self._renderer_model_id = id(model)

  def is_running(self) -> bool:
    """Poll for a new frame; returns ``False`` when the window is closed."""
    if not self._is_running:
      return False
    self._is_running = self._viewer.NewFrame()
    return self._is_running

  def sync(
      self,
      model: mujoco.MjModel,
      data: mujoco.MjData,
  ) -> None:
    """Render the scene and present it to the window.

    Args:
      model: The MuJoCo model provided by the simulation.
      data: The MuJoCo data provided by the simulation.
    """
    self._sync_renderer(model)
    self._viewer.Present(
        model,
        data,
        self.perturb,
        self.camera,
        self.vis_options,
        self.render_flags.flags,
    )

  def stop(self) -> None:
    """Stop the viewer."""
    self._is_running = False

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
