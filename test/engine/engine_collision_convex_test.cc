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

// Tests for engine/engine_collision_convex.c.

#include <cstddef>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::NotNull;

static const char* const kFramelessContactPath =
    "engine/testdata/collision_convex/frameless_contact.xml";
static const char* const kFramelessContactHfieldPath =
    "engine/testdata/collision_convex/frameless_contact_hfield.xml";
static const char* const kCylinderBoxPath =
    "engine/testdata/collision_convex/cylinder_box.xml";

using MjcConvexTest = MujocoTest;

TEST_F(MjcConvexTest, FramelessContact) {
  const std::string xml_path = GetTestDataFilePath(kFramelessContactPath);
  char error[1024];
  const std::size_t error_sz = 1024;
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, error_sz);
  // Loading used to fail with "engine error: xaxis of contact frame undefined".
  EXPECT_THAT(model, NotNull()) << "Failed to load model: " << error;
  mj_deleteModel(model);
}

TEST_F(MjcConvexTest, FramelessContactHfield) {
  const std::string xml_path = GetTestDataFilePath(kFramelessContactHfieldPath);
  char error[1024];
  const std::size_t error_sz = 1024;
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, error_sz);
  // Loading used to fail with "engine error: xaxis of contact frame undefined".
  EXPECT_THAT(model, NotNull()) << "Failed to load model: " << error;
  mj_deleteModel(model);
}

TEST_F(MjcConvexTest, CylinderBox) {
  const std::string xml_path = GetTestDataFilePath(kCylinderBoxPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  ASSERT_THAT(model, NotNull());
  mjData* data = mj_makeData(model);

  // with multiCCD enabled, should find 5 contacts
  mj_forward(model, data);
  EXPECT_EQ(data->ncon, 5);

  // with multiCCD disabled, should find 1 contact
  model->opt.enableflags &= ~mjENBL_MULTICCD;
  mj_forward(model, data);
  EXPECT_EQ(data->ncon, 1);

  mj_deleteData(data);
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
