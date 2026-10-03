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

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include <mujoco/mujoco.h>
#include <pxr/base/plug/registry.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/prim.h>
#include "rules_cc/cc/runfiles/runfiles.h"

int main(int argc, char** argv) {
  if (argc != 6) return 1;
  using rules_cc::cc::runfiles::Runfiles;
  std::unique_ptr<Runfiles> runfiles(Runfiles::CreateForTest());
  if (!runfiles) return 1;
  for (int index = 1; index <= 4; ++index) {
    auto plugins = pxr::PlugRegistry::GetInstance().RegisterPlugins(
        runfiles->Rlocation(argv[index]));
    if (plugins.empty()) return 1;
  }
  std::filesystem::path model_path =
      std::filesystem::path(std::getenv("TEST_TMPDIR")) / "model.xml";
  std::ofstream(model_path) << R"(<mujoco><worldbody><body name="sphere">
      <freejoint/><geom type="sphere" size="0.1"/>
      </body></worldbody></mujoco>)";
  auto stage = pxr::UsdStage::Open(model_path.string());
  if (!stage || stage->GetPseudoRoot().GetChildren().empty()) return 1;
  auto usd_path = model_path.replace_extension("usda");
  if (!stage->Export(usd_path.string())) return 1;
  mj_loadPluginLibrary(runfiles->Rlocation(argv[5]).c_str());
  char error[1024] = {};
  mjSpec* spec = mj_parse(usd_path.c_str(), "model/usd", nullptr, error, sizeof(error));
  mjModel* model = spec ? mj_compile(spec, nullptr) : nullptr;
  if (spec && !model) std::fprintf(stderr, "%s\n", mjs_getError(spec));
  mj_deleteSpec(spec);
  if (!model) {
    std::fprintf(stderr, "%s\n", error);
    return 1;
  }
  mjData* data = mj_makeData(model);
  mj_step(model, data);
  bool advanced = data->time > 0;
  mj_deleteData(data);
  mj_deleteModel(model);
  return advanced ? 0 : 1;
}
