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
#include <memory>
#include <string>

#include <benchmark/benchmark.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <absl/base/attributes.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::Eq;
using ::testing::NotNull;


static void run_parse_benchmark(const std::string xml_path,
                                benchmark::State& state) {
  MujocoErrorTestGuard guard;  // Fail test if there are any mujoco errors

  int vfs_errno = 0;
  std::string vfs_errmsg = "";
  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());

  if ((vfs_errno = mj_addFileVFS(vfs.get(), "", xml_path.data()))) {
    if (vfs_errno == 1) {
      vfs_errmsg = "VFS is full";  // should not occur
    } else if (vfs_errno == 2) {
      vfs_errmsg = "Repeated name in VFS";  // should not occur
    } else {
      vfs_errmsg = "File not found";
    }
  }

  ASSERT_THAT(vfs_errno, Eq(0)) << "Failed to add file to VFS: " << vfs_errmsg;

  // load once to warm up filesystem and compiler cache
  std::array<char, 1024> error;
  mjModel* model =
      mj_loadXML(xml_path.data(), vfs.get(), error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  for (auto s : state) {
    mjModel* model = mj_loadXML(xml_path.data(), vfs.get(), 0, 0);
    mj_deleteModel(model);
  }
  state.SetLabel(xml_path);
  mj_deleteVFS(vfs.get());
}

// Use ABSL_ATTRIBUTE_NO_TAIL_CALL to make sure the benchmark functions appear
// separately in CPU profiles (and don't get replaced with raw calls to
// run_parse_benchmark).

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_ParseFlagPlugin(benchmark::State& state) {
  run_parse_benchmark(GetModelPath("flex/flag.xml"), state);
}
BENCHMARK(BM_ParseFlagPlugin);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_ParseFlag(benchmark::State& state) {
  run_parse_benchmark(GetModelPath("../test/benchmark/testdata/flag.xml"),
                      state);
}
BENCHMARK(BM_ParseFlag);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_ParseHumanoid(benchmark::State& state) {
  run_parse_benchmark(GetModelPath("humanoid/humanoid.xml"), state);
}
BENCHMARK(BM_ParseHumanoid);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_ParseHumanoid100(benchmark::State& state) {
  run_parse_benchmark(GetModelPath("humanoid/humanoid100.xml"), state);
}
BENCHMARK(BM_ParseHumanoid100);

}  // namespace
}  // namespace mujoco
