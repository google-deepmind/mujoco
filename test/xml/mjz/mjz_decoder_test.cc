// Copyright 2026 DeepMind Technologies Limited
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

#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjspec.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using MjzTest = MujocoTest;

using ::testing::IsNull;
using ::testing::Not;
using ::testing::NotNull;
using ::testing::StrEq;

TEST_F(MjzTest, Parse) {
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  std::string filepath = GetTestDataFilePath("testdata/model.mjz");
  char err[1000] = "";
  mjSpec* spec = mj_parse(filepath.c_str(), "", &vfs, err, sizeof(err));
  EXPECT_THAT(spec, NotNull());
  EXPECT_THAT(err, StrEq(""));
  mjModel* model = mj_compile(spec, &vfs);
  EXPECT_THAT(model, NotNull()) << mjs_getError(spec);
  mj_deleteModel(model);
  mj_deleteSpec(spec);
  mj_deleteVFS(&vfs);
}

TEST_F(MjzTest, InvalidPath) {
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  std::string filepath = GetTestDataFilePath("testdata/not_a_file.mjz");
  char err[1000] = "";
  mjSpec* spec = mj_parse(filepath.c_str(), "", nullptr, err, sizeof(err));
  EXPECT_THAT(spec, IsNull());
  EXPECT_THAT(err, Not(StrEq("")));
  mj_deleteVFS(&vfs);
}

}  // namespace
}  // namespace mujoco
