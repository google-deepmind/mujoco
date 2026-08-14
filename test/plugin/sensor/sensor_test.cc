// Copyright 2023 DeepMind Technologies Limited
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

// Tests for sensor plugins.

#include <string>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using SensorPluginTest = MujocoTest;

static const char* const kTOuchGridPath =
    "plugin/sensor/testdata/touch_grid_test.xml";

TEST_F(SensorPluginTest, ValidAttributes) {
  const std::string xml_path = GetTestDataFilePath(kTOuchGridPath);
  char error[1024] = {0};
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(model, testing::NotNull()) << error;
  int nchannel = 3;
  int size_x = 6;
  int size_y = 2;
  EXPECT_THAT(model->sensor_dim[0], nchannel * size_x * size_y);
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
