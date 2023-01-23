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
  std::string file1 = "activation.xml";
  std::string file2 = "damper.xml";
  std::string file3 = "refsite.xml";

  std::FILE* fp1 = std::fopen((dir + file1).c_str(), "r");
  ASSERT_THAT(fp1, NotNull()) << "Input file1 missing.";
  std::fclose(fp1);

  std::FILE* fp2 = std::fopen((dir + file2).c_str(), "r");
  ASSERT_THAT(fp2, NotNull()) << "Input file2 missing.";
  std::fclose(fp2);

  std::FILE* fp3 = std::fopen((dir + file3).c_str(), "r");
  ASSERT_THAT(fp3, NotNull()) << "Input file3 missing.";
  std::fclose(fp3);

  auto mj_vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(mj_vfs.get());

  EXPECT_THAT(mj_vfs->nfile, 0);
  EXPECT_THAT(mj_addFileVFS(mj_vfs.get(), dir.c_str(), file1.c_str()), 0);
  EXPECT_THAT(mj_vfs->nfile, 1);
  EXPECT_THAT(mj_vfs->filename[0], file1);

  EXPECT_THAT(mj_addFileVFS(mj_vfs.get(), dir.c_str(), file2.c_str()), 0);
  EXPECT_THAT(mj_vfs->nfile, 2);
  EXPECT_THAT(mj_vfs->filename[0], file1);
  EXPECT_THAT(mj_vfs->filename[1], file2);

  EXPECT_THAT(mj_addFileVFS(mj_vfs.get(), dir.c_str(), file3.c_str()), 0);
  EXPECT_THAT(mj_vfs->nfile, 3);
  EXPECT_THAT(mj_vfs->filename[0], file1);
  EXPECT_THAT(mj_vfs->filename[1], file2);
  EXPECT_THAT(mj_vfs->filename[2], file3);

  mj_deleteFileVFS(mj_vfs.get(), file1.c_str());
  EXPECT_THAT(mj_vfs->nfile, 2);
  EXPECT_THAT(mj_vfs->filename[0], file2);
  EXPECT_THAT(mj_vfs->filename[1], file3);

  mj_deleteFileVFS(mj_vfs.get(), file3.c_str());
  EXPECT_THAT(mj_vfs->nfile, 1);
  EXPECT_THAT(mj_vfs->filename[0], file2);

  mj_deleteFileVFS(mj_vfs.get(), file2.c_str());
  EXPECT_THAT(mj_vfs->nfile, 0);

  mj_deleteVFS(mj_vfs.get());
}

}  // namespace
}  // namespace mujoco
