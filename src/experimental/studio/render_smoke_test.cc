// Copyright 2026 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstddef>
#include <cstdio>
#include <memory>
#include <vector>

#include <mujoco/mujoco.h>
#include "experimental/studio/hal/filament_renderer.h"
#include "experimental/studio/hal/graphics_mode.h"
#include "experimental/studio/io/resources.h"
#include "experimental/studio/sim/model_holder.h"

int main() {
  mujoco::studio::RegisterResourceProviders();
  mjSpec* spec = mj_makeSpec();
  const double clear_color[] = {1.0, 1.0, 1.0, 1.0};
  mjsNumeric* numeric = mjs_addNumeric(spec);
  mjs_setName(numeric->element, "filament.clearColor");
  numeric->size = 4;
  mjs_setDouble(numeric->data, clear_color, numeric->size);
  mjsGeom* sphere = mjs_addGeom(mjs_findBody(spec, "world"), nullptr);
  sphere->type = mjGEOM_SPHERE;
  sphere->size[0] = 0.3;
  sphere->rgba[0] = 0.9f;
  sphere->rgba[1] = 0.1f;
  sphere->rgba[2] = 0.1f;
  sphere->rgba[3] = 1.0f;
  auto holder = mujoco::studio::ModelHolder::FromSpec(spec);
  if (!holder) {
    std::fputs("Cannot compile the render model.\n", stderr);
    return 1;
  }
  constexpr int width = 64;
  constexpr int height = 64;
  holder->model()->vis.global.offwidth = width;
  holder->model()->vis.global.offheight = height;
  mj_forward(holder->model(), holder->data());
  mujoco::studio::FilamentRenderer renderer(
      nullptr, mujoco::studio::GraphicsMode::FilamentVulkan);
  renderer.Init(holder->model());
  mjvCamera camera;
  mjv_defaultFreeCamera(holder->model(), &camera);
  camera.distance = 2.0;
  std::vector<std::byte> pixels(width * height * 3);
  renderer.Render(holder->model(), holder->data(), nullptr, &camera, nullptr,
                  width, height, pixels);
  bool background = false;
  bool geometry = false;
  for (std::size_t i = 0; i < pixels.size(); i += 3) {
    const int red = std::to_integer<int>(pixels[i]);
    const int green = std::to_integer<int>(pixels[i + 1]);
    const int blue = std::to_integer<int>(pixels[i + 2]);
    background |= red > 200 && green > 200 && blue > 200;
    geometry |= red > green + 20 && red > blue + 20;
  }
  if (!background || !geometry) {
    std::fprintf(stderr,
                 "The rendered frame must contain the background (%d) and "
                 "red sphere (%d).\n", background, geometry);
    return 1;
  }
}
