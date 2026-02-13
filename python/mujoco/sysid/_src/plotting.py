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

"""Plotting utilities."""

from __future__ import annotations

from collections.abc import Sequence

import mujoco
import numpy as np


def render_rollout(
    model: mujoco.MjModel | Sequence[mujoco.MjModel],
    data: mujoco.MjData,
    state: np.ndarray,
    framerate: int,
    camera: str | int = -1,
    width: int = 640,
    height: int = 480,
    light_pos: Sequence[float] | None = None,
) -> list[np.ndarray]:
  """Renders a rollout or batch of rollouts.

  Args:
    model: Single model or list of models (one per batch).
    data: MjData scratch object.
    state: State array of shape (nbatch, nsteps, nstate).
    framerate: Frames per second to render.
    camera: Camera name or ID.
    width: Image width.
    height: Image height.
    light_pos: Optional light position [x, y, z] to add a spotlight.

  Returns:
    List of rendered frames (numpy arrays).
  """
  nbatch = state.shape[0]

  if isinstance(model, mujoco.MjModel):
    models_list = [model] * nbatch
  else:
    models_list = list(model)
    if len(models_list) == 1:
      models_list = models_list * nbatch
    else:
      assert len(models_list) == nbatch

  # Visual options
  vopt = mujoco.MjvOption()
  vopt.geomgroup[3] = 1  # Show visualization geoms

  pert = mujoco.MjvPerturb()
  catmask = mujoco.mjtCatBit.mjCAT_DYNAMIC.value

  # Simulate and render.
  frames = []

  with mujoco.Renderer(models_list[0], height=height, width=width) as renderer:
    for i in range(state.shape[1]):
      # Check if we should capture this frame based on framerate
      if len(frames) < i * models_list[0].opt.timestep * framerate:
        for j in range(state.shape[0]):
          # Set state
          mujoco.mj_setState(
              models_list[j],
              data,
              state[j, i, :],
              mujoco.mjtState.mjSTATE_FULLPHYSICS.value,
          )
          mujoco.mj_forward(models_list[j], data)

          # Use first model to make the scene, add subsequent models
          if j == 0:
            renderer.update_scene(data, camera, scene_option=vopt)
          else:
            mujoco.mjv_addGeoms(
                models_list[j], data, vopt, pert, catmask, renderer.scene
            )

        # Add light, if requested
        if light_pos is not None:
          if renderer.scene.nlight < 100:  # check limit
            light = renderer.scene.lights[renderer.scene.nlight]
            light.ambient = [0, 0, 0]
            light.attenuation = [1, 0, 0]
            light.castshadow = 1
            light.cutoff = 45
            light.diffuse = [0.8, 0.8, 0.8]
            light.dir = [0, 0, -1]
            light.type = mujoco.mjtLightType.mjLIGHT_SPOT
            light.exponent = 10
            light.headlight = 0
            light.specular = [0.3, 0.3, 0.3]
            light.pos = light_pos
            renderer.scene.nlight += 1

        # Render and add the frame.
        pixels = renderer.render()
        frames.append(pixels)
  return frames
