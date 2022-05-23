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

// Tests for xml/xml_api.cc.

#include <cstddef>
#include <cstring>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::IsNull;
using ::testing::NotNull;

// ---------------------------- test mj_loadXML --------------------------------

using LoadXmlTest = MujocoTest;

TEST_F(LoadXmlTest, EmptyModel) {
  static constexpr char xml[] = "<mujoco/>";
  mjModel* model = LoadModelFromString(xml, 0, 0);
  ASSERT_THAT(model, NotNull());
  EXPECT_EQ(model->nq, 0);
  EXPECT_EQ(model->nv, 0);
  EXPECT_EQ(model->nu, 0);
  EXPECT_EQ(model->na, 0);
  EXPECT_EQ(model->nbody, 1);  // worldbody exists even in empty model

  mjData* data = mj_makeData(model);
  EXPECT_THAT(data, NotNull());
  mj_step(model, data);
  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(LoadXmlTest, InvalidXmlFailsToLoad) {
  static constexpr char invalid_xml[] = "<mujoc";
  char error[1024];
  size_t error_sz = 1024;
  mjModel* model = LoadModelFromString(invalid_xml, error, error_sz);
  EXPECT_THAT(model, IsNull()) << "Expected model loading to fail.";
  EXPECT_GT(std::strlen(error), 0);
  if (model) {
    mj_deleteModel(model);
  }
}
// TODO(nimrod): Add more tests for mj_loadXML.

}  // namespace
}  // namespace mujoco
