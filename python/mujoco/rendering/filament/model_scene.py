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
"""Manages Lights and Renderables in an Scene based on mjModel/mjData."""

import mujoco
from mujoco import mjrf


class ModelScene:
  """Manages Lights and Renderables in an Scene based on mjModel/mjData."""

  def __init__(
      self,
      ctx: mjrf.Context,
      model: mujoco.MjModel,
      scene: mjrf.Scene | None = None,
  ):
    self._objects = mjrf.ModelObjects(ctx, model)
    self._scene = scene or ctx.create_scene(mjrf.SceneParams())
    self._scene.configure_from_model(model)
    self._lights = mjrf.ModelLights(self._scene, self._objects)
    self._renderables = mjrf.ModelRenderables(self._scene, self._objects)

  def update(self, data: mujoco.MjData):
    """Updates the lights and renderables in the scene from the mjData."""
    self._lights.update(data)
    self._renderables.update(data)

  @property
  def scene(self) -> mjrf.Scene:
    """Returns the scene."""
    return self._scene

  def get_light(self, index: int) -> mjrf.Light:
    """Returns the light with the given index in the mjModel."""
    return self._lights.get_light(index)

  def get_renderable(
      self,
      obj_type: mujoco.mjtObj,
      obj_index: int,
      *,
      sub_index: int = -1,
  ) -> mjrf.Renderable:
    """Returns the renderable corresponding to the given mjModel object."""
    return self._renderables.get_renderable(obj_type, obj_index, sub_index)
