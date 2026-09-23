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
"""Renders multiple views of a MuJoCo model with Filament."""

from collections.abc import Sequence

from absl import app
import mujoco
import mujoco._render_filament as mjrf
from mujoco.experimental.studio import window
from mujoco.rendering.filament import renderer


def main(argv: Sequence[str]) -> None:
  if len(argv) < 2:
    raise app.UsageError("Please provide a path to a model XML file.")

  # create an mjModel and mjData from a path to an xml file
  model = mujoco.MjModel.from_xml_path(argv[1])
  data = mujoco.MjData(model)

  modes = [
      mjrf.DrawMode.DRAW_MODE_DEFAULT,
      mjrf.DrawMode.DRAW_MODE_SEGMENTATION_BY_ID,
      mjrf.DrawMode.DRAW_MODE_DEPTH,
  ]
  ncam = max(model.ncam, 1)

  # create a window
  win = window.Window(
      "Filament Multiview", 256 * ncam, 256 * len(modes), "opengl_headless"
  )

  # create the filament render context
  ctx = mjrf.Context(
      mjrf.ContextConfig(graphics_api=mjrf.GraphicsApi.GRAPHICS_API_OPENGL)
  )

  # create a renderer
  r = renderer.Renderer(ctx)

  # create GPU objects (textures, meshes, etc.) from the model
  model_objects = mjrf.ModelObjects(ctx, model)

  # add model lights and renderables to a scene
  model_lights = mjrf.ModelLights(r.scene("scn"), model_objects)
  model_renderables = mjrf.ModelRenderables(r.scene("scn"), model_objects)

  # define multiple views
  dx = 1.0 / ncam
  dy = 1.0 / len(modes)
  for y, mode in enumerate(modes):
    for x in range(ncam):
      vp = (x * dx, y * dy, dx, dy)
      r.view(
          f"v_{x}_{y}",
          scene="scn",
          target="out",
          viewport=vp,
          draw_mode=mode,
      )

  while win.NewFrame():
    # simulate one step
    mujoco.mj_step(model, data)

    # update the lights and renderables
    model_lights.update(data)
    model_renderables.update(data)

    # update the cameras
    for y, _ in enumerate(modes):
      for x in range(ncam):
        vcam = mujoco.MjvCamera()
        if model.ncam == 0:
          vcam.type = int(mujoco.mjtCamera.mjCAMERA_FREE)
          mujoco.mjv_defaultFreeCamera(model, vcam)
        else:
          vcam.type = int(mujoco.mjtCamera.mjCAMERA_FIXED)
          vcam.fixedcamid = x
        r.update_camera(
            f"v_{x}_{y}", mujoco.mjv_camera2GLCamera(model, data, vcam)
        )

    # ensure our output buffer is of the right size
    r.target("out", (win.GetWidth(), win.GetHeight()))

    # render the scene
    r.render()

    # present the output image to the window
    win.Present(r.get_image("out").pixels)


if __name__ == "__main__":
  app.run(main)
