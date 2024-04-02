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

#include <array>
#include <cstdio>
#include <memory>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_resource.h"
#include "src/engine/engine_vfs.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::NotNull;
using EngineVfsTest = MujocoTest;

TEST_F(EngineVfsTest, AddFile) {
  constexpr char path[] = "engine/testdata/actuation/";
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

  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());

  EXPECT_THAT(vfs->nfile, 0);
  EXPECT_THAT(mj_addFileVFS(vfs.get(), dir.c_str(), file1.c_str()), 0);
  EXPECT_THAT(vfs->nfile, 1);
  EXPECT_THAT(vfs->filename[0], file1);

  EXPECT_THAT(mj_addFileVFS(vfs.get(), dir.c_str(), file2.c_str()), 0);
  EXPECT_THAT(vfs->nfile, 2);
  EXPECT_THAT(vfs->filename[0], file1);
  EXPECT_THAT(vfs->filename[1], file2);

  EXPECT_THAT(mj_addFileVFS(vfs.get(), dir.c_str(), file3.c_str()), 0);
  EXPECT_THAT(vfs->nfile, 3);
  EXPECT_THAT(vfs->filename[0], file1);
  EXPECT_THAT(vfs->filename[1], file2);
  EXPECT_THAT(vfs->filename[2], file3);

  mj_deleteFileVFS(vfs.get(), file1.c_str());
  EXPECT_THAT(vfs->nfile, 2);
  EXPECT_THAT(vfs->filename[0], file2);
  EXPECT_THAT(vfs->filename[1], file3);

  mj_deleteFileVFS(vfs.get(), file3.c_str());
  EXPECT_THAT(vfs->nfile, 1);
  EXPECT_THAT(vfs->filename[0], file2);

  mj_deleteFileVFS(vfs.get(), file2.c_str());
  EXPECT_THAT(vfs->nfile, 0);

  mj_deleteVFS(vfs.get());
}

TEST_F(EngineVfsTest, AddBuffer) {
  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());
  std::string buffer = "<mujoco/>";
  mj_addBufferVFS(vfs.get(), "model", static_cast<const void*>(buffer.c_str()),
                  buffer.size());
  std::array<char, 1024> error;
  mjModel* model = mj_loadXML("model", vfs.get(), error.data(), error.size());
  EXPECT_THAT(model, NotNull());
  mj_deleteModel(model);
  mj_deleteVFS(vfs.get());
}

TEST_F(EngineVfsTest, Timestamps) {
  static constexpr char cube[] = R"(
  v -0.500000 -0.500000  0.500000
  v  0.500000 -0.500000  0.500000
  v -0.500000  0.500000  0.500000
  v  0.500000  0.500000  0.500000
  v -0.500000  0.500000 -0.500000
  v  0.500000  0.500000 -0.500000
  v -0.500000 -0.500000 -0.500000
  v  0.500000 -0.500000 -0.500000)";

  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());
  mj_addBufferVFS(vfs.get(), "cube.obj", cube, sizeof(cube));

  mjResource* resource = mju_openVfsResource("cube.obj", vfs.get());

  // same timestamps
  EXPECT_EQ(mju_isModifiedResource(resource, resource->timestamp), 0);

  // different timestamps
  EXPECT_EQ(mju_isModifiedResource(resource, "QQ=="), 1);

  mju_closeResource(resource);
  mj_deleteVFS(vfs.get());
}

}  // namespace
}  // namespace mujoco
