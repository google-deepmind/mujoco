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

// Benchmarks for sleep filtering on static-root subtrees.

#include <string>

#include <benchmark/benchmark.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

void BM_SubtreeContactSleep(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  int count = state.range(0);
  int sleep = state.range(1);  // 0: static, 1: asleep, 2: awake
  bool chain = state.range(2);
  std::string xml = R"(
    <mujoco>
      <option gravity="0 0 0"><flag sleep="enable" contact="disable"/></option>
      <worldbody><body name="root">
  )";
  for (int b = 0; b < count; b++) {
    bool movable = sleep && (!chain || b == count-1);
    xml += movable && sleep == 1 ? "<body sleep=\"init\">" : "<body>";
    if (movable) {
      xml += "<joint type=\"slide\"/><geom size=\".01\"/>";
    }
    if (!chain) {
      xml += "</body>";
    }
  }
  if (chain) {
    for (int b = 0; b < count; b++) {
      xml += "</body>";
    }
  }
  // Keep sleep filtering enabled even when the queried subtree is entirely awake or static.
  xml += R"(</body><body sleep="init" pos="100 0 0"><joint/><geom size=".01"/></body>
             </worldbody><sensor><contact subtree1="root"/></sensor></mujoco>)";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  if (!model) {
    state.SkipWithError(error);
    return;
  }
  MjDataPtr data = MakeData(model);
  mj_forward(model.get(), data.get());

  for (auto _ : state) {
    mj_sensorAcc(model.get(), data.get());
    benchmark::DoNotOptimize(data->sensordata);
  }
}
BENCHMARK(BM_SubtreeContactSleep)
    ->ArgsProduct({{32, 128, 256}, {0, 1, 2}, {0, 1}})
    ->ArgNames({"bodies", "sleep", "chain"});

}  // namespace
}  // namespace mujoco
