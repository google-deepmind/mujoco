// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// A benchmark for parsing and compiling models from XML.

#include <array>

#include <benchmark/benchmark.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <absl/base/attributes.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::NotNull;

static void run_parse_benchmark(const char *model_path, benchmark::State& state) {
  MujocoErrorTestGuard guard;  // Fail test if there are any mujoco errors

  const std::string xml_path = GetModelPath(model_path);
  std::array<char, 1024> error;
  for (auto s : state) {
    // TODO(nimrod): Load the models from VFS rather than from disk, to
    // limit the benchmark to the parsing and model compilation speed.
    mjModel* model =
      mj_loadXML(xml_path.data(), nullptr, error.data(), error.size());
    ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();
    mj_deleteModel(model);
  }
  state.SetLabel(model_path);
}

// Use ABSL_ATTRIBUTE_NO_TAIL_CALL to make sure the benchmark functions appear
// separately in CPU profiles (and don't get replaced with raw calls to
// run_parse_benchmark).

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_ParseCloth(benchmark::State& state) {
  run_parse_benchmark("composite/cloth.xml", state);
}
BENCHMARK(BM_ParseCloth);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_ParseHumanoid(benchmark::State& state) {
  run_parse_benchmark("humanoid/humanoid.xml", state);
}
BENCHMARK(BM_ParseHumanoid);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_ParseHumanoid100(benchmark::State& state) {
  run_parse_benchmark("humanoid100/humanoid100.xml", state);
}
BENCHMARK(BM_ParseHumanoid100);

}  // namespace
}  // namespace mujoco
