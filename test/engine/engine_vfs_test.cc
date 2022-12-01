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

#include <cstddef>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::NotNull;
using EngineVfsTest = MujocoTest;

TEST_F(EngineVfsTest, AddFileVFS) {
  constexpr char path[] = "engine/testdata/";
  const std::string dir = GetTestDataFilePath(path);
  std::string file = "refsite.xml";

  std::FILE* fp = std::fopen((dir + file).c_str(), "r");
  ASSERT_THAT(fp, NotNull()) << "Input file missing.";
  std::fclose(fp);

  auto mj_vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(mj_vfs.get());
  EXPECT_THAT(mj_addFileVFS(mj_vfs.get(), dir.c_str(), file.c_str()), 0);
  EXPECT_THAT(mj_vfs->nfile, 1);
  mj_deleteFileVFS(mj_vfs.get(), file.c_str());
  mj_deleteVFS(mj_vfs.get());
}

}  // namespace
}  // namespace mujoco
