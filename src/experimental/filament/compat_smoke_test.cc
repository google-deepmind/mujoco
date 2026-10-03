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

#include <cstring>

#include <mujoco/mujoco.h>

int main() {
  mjrContext context;
  mjr_defaultContext(&context);
  mjrRendererInfo info;
  mjr_getRendererInfo(&info);
  if (std::strcmp(info.renderer, "filament") != 0) return 1;
  mjr_freeContext(&context);
  mjSpec* spec = mj_makeSpec();
  mjModel* model = mj_compile(spec, nullptr);
  mj_deleteSpec(spec);
  if (!model) return 1;
  mjData* data = mj_makeData(model);
  mj_step(model, data);
  bool advanced = data->time > 0;
  mj_deleteData(data);
  mj_deleteModel(model);
  return advanced ? 0 : 1;
}
