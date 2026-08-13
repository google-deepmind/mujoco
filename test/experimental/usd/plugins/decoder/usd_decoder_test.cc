// Copyright 2026 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using UsdDecoderTest = MujocoTest;
using ::testing::NotNull;

TEST_F(UsdDecoderTest, NewtonMimicEnabledControlsEquality) {
  mj_loadPluginLibrary(USD_DECODER_PLUGIN_PATH);

  const std::string path = GetTestDataFilePath(
      "experimental/usd/plugins/decoder/testdata/mimic_enabled.usda");
  char error[1024] = {};
  mjSpec* spec = mj_parse(path.c_str(), nullptr, nullptr, error, sizeof(error));
  ASSERT_THAT(spec, NotNull()) << error;

  mjModel* model = mj_compile(spec, nullptr);
  ASSERT_THAT(model, NotNull()) << mjs_getError(spec);
  ASSERT_EQ(model->neq, 1);
  EXPECT_EQ(model->eq_active0[0], 0);

  mj_deleteModel(model);
  mj_deleteSpec(spec);
}

}  // namespace
}  // namespace mujoco
