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
"""A simple example of rendering a MuJoCo model with Filament."""

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

  # create a window
  win = window.Window("Filament Render", 512, 512, "opengl_headless")

  # create the filament render context
  ctx = mjrf.Context(
      mjrf.ContextConfig(graphics_api=mjrf.GraphicsApi.GRAPHICS_API_OPENGL)
  )

  # create a renderer
  r = renderer.Renderer(ctx)

  # create GPU objects (textures, meshes, etc.) from the model
  model_objects = mjrf.ModelObjects(ctx, model)

  # add model lights and renderables to a scene
  scene = r.scene("scn")
  scene.configure_from_model(model)
  model_lights = mjrf.ModelLights(scene, model_objects)
  model_renderables = mjrf.ModelRenderables(scene, model_objects)

  vcam = mujoco.MjvCamera()
  mujoco.mjv_defaultFreeCamera(model, vcam)
  vcam.type = int(mujoco.mjtCamera.mjCAMERA_FIXED)
  vcam.fixedcamid = 0

  # define a single view that connects a scene and camera to an output
  r.view(
      "v1",
      scene="scn",
      target="out",
  )

  while win.NewFrame():
    # simulate one step
    mujoco.mj_step(model, data)

    # update the lights and renderables
    model_lights.update(data)
    model_renderables.update(data)

    # update the camera
    r.update_camera("v1", mujoco.mjv_camera2GLCamera(model, data, vcam))

    # ensure output buffer is of right size (in case of window resize)
    r.target("out", (win.GetWidth(), win.GetHeight()))

    # render the scene
    r.render()

    # present the output image to the window
    win.Present(r.get_image("out").pixels)


if __name__ == "__main__":
  app.run(main)
