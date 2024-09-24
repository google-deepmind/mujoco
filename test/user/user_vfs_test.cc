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
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "src/user/user_resource.h"
#include "src/user/user_vfs.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::NotNull;
using UserVfsTest = MujocoTest;

static bool HasFile(const mjVFS* vfs, const std::string& filename) {
  mjResource* resource = mju_openResource("", filename.c_str(), vfs, nullptr, 0);
  bool result = resource != nullptr;
  mju_closeResource(resource);
  return result;
}

TEST_F(UserVfsTest, AddFile) {
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

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  EXPECT_FALSE(HasFile(&vfs, file1));
  EXPECT_THAT(mj_addFileVFS(&vfs, dir.c_str(), file1.c_str()), 0);
  EXPECT_TRUE(HasFile(&vfs, file1));

  EXPECT_THAT(mj_addFileVFS(&vfs, dir.c_str(), file2.c_str()), 0);
  EXPECT_TRUE(HasFile(&vfs, file1));
  EXPECT_TRUE(HasFile(&vfs, file2));

  EXPECT_THAT(mj_addFileVFS(&vfs, dir.c_str(), file3.c_str()), 0);
  EXPECT_TRUE(HasFile(&vfs, file1.c_str()));
  EXPECT_TRUE(HasFile(&vfs, file2.c_str()));
  EXPECT_TRUE(HasFile(&vfs, file3.c_str()));

  mj_deleteFileVFS(&vfs, file1.c_str());
  EXPECT_FALSE(HasFile(&vfs, file1.c_str()));
  EXPECT_TRUE(HasFile(&vfs, file2.c_str()));
  EXPECT_TRUE(HasFile(&vfs, file3.c_str()));


  mj_deleteFileVFS(&vfs, file3.c_str());
  EXPECT_FALSE(HasFile(&vfs, file1.c_str()));
  EXPECT_TRUE(HasFile(&vfs, file2.c_str()));
  EXPECT_FALSE(HasFile(&vfs, file3.c_str()));

  mj_deleteFileVFS(&vfs, file2.c_str());
  EXPECT_FALSE(HasFile(&vfs, file1.c_str()));
  EXPECT_FALSE(HasFile(&vfs, file2.c_str()));
  EXPECT_FALSE(HasFile(&vfs, file3.c_str()));

  mj_deleteVFS(&vfs);
}

TEST_F(UserVfsTest, AddFileStripPath) {
  mjVFS vfs;
  mj_defaultVFS(&vfs);

  constexpr char path[] = "engine/testdata/actuation/";
  const std::string dir = GetTestDataFilePath(path);
  std::string file1 = "activation.xml";
  mj_addFileVFS(&vfs, dir.c_str(), file1.c_str());

  EXPECT_TRUE(HasFile(&vfs, file1));
  EXPECT_TRUE(HasFile(&vfs, dir + file1));
  EXPECT_TRUE(HasFile(&vfs, "some/dir/" + file1));
  EXPECT_TRUE(HasFile(&vfs, "some/dir\\" + file1));

  mj_deleteVFS(&vfs);
}

TEST_F(UserVfsTest, AddFileRepeat) {
  mjVFS vfs;
  mj_defaultVFS(&vfs);

  constexpr char path[] = "engine/testdata/actuation/";
  const std::string dir = GetTestDataFilePath(path);
  std::string file1 = "activation.xml";
  mj_addFileVFS(&vfs, dir.c_str(), file1.c_str());

  EXPECT_TRUE(HasFile(&vfs, file1));
  EXPECT_THAT(mj_addFileVFS(&vfs, "dir/", file1.c_str()), 2);

  mj_deleteVFS(&vfs);
}

TEST_F(UserVfsTest, DeleteFile) {
  mjVFS vfs;
  mj_defaultVFS(&vfs);

  constexpr char path[] = "engine/testdata/actuation/";
  const std::string dir = GetTestDataFilePath(path);
  std::string file1 = "activation.xml";
  mj_addFileVFS(&vfs, dir.c_str(), file1.c_str());

  EXPECT_TRUE(HasFile(&vfs, file1));
  EXPECT_THAT(mj_deleteFileVFS(&vfs, file1.c_str()), 0);
  EXPECT_FALSE(HasFile(&vfs, file1));

  mj_deleteVFS(&vfs);
}

TEST_F(UserVfsTest, DeleteFileStripPath) {
  mjVFS vfs;
  mj_defaultVFS(&vfs);

  constexpr char path[] = "engine/testdata/actuation/";
  const std::string dir = GetTestDataFilePath(path);
  std::string file = "activation.xml";
  std::string fileUpper = "Activation.xml";
  mj_addFileVFS(&vfs, dir.c_str(), file.c_str());

  EXPECT_TRUE(HasFile(&vfs, file));
  EXPECT_THAT(mj_deleteFileVFS(&vfs, ("dir\\" + fileUpper).c_str()), 0);
  EXPECT_FALSE(HasFile(&vfs, file));

  mj_deleteVFS(&vfs);
}

TEST_F(UserVfsTest, DeleteFileRepeat) {
  mjVFS vfs;
  mj_defaultVFS(&vfs);

  constexpr char path[] = "engine/testdata/actuation/";
  const std::string dir = GetTestDataFilePath(path);
  std::string file = "activation.xml";
  mj_addFileVFS(&vfs, dir.c_str(), file.c_str());

  EXPECT_TRUE(HasFile(&vfs, file));
  EXPECT_THAT(mj_deleteFileVFS(&vfs, file.c_str()), 0);
  EXPECT_FALSE(HasFile(&vfs, file));
  EXPECT_THAT(mj_deleteFileVFS(&vfs, file.c_str()), -1);

  mj_deleteVFS(&vfs);
}


TEST_F(UserVfsTest, AddBuffer) {
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  std::string buffer = "<mujoco/>";
  mj_addBufferVFS(&vfs, "model", static_cast<const void*>(buffer.c_str()),
                  buffer.size());
  std::array<char, 1024> error;
  mjModel* model = mj_loadXML("model", &vfs, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();
  mj_deleteModel(model);
  mj_deleteVFS(&vfs);
}

TEST_F(UserVfsTest, AddBufferRepeat) {
  mjVFS vfs;
  mj_defaultVFS(&vfs);
    std::string buffer = "<mujoco/>";
    const void* ptr = static_cast<const void*>(buffer.c_str());
  mj_addBufferVFS(&vfs, "model", ptr, buffer.size());
  int result = mj_addBufferVFS(&vfs, "model", ptr, buffer.size());
  EXPECT_EQ(result, 2);
  mj_deleteVFS(&vfs);
}

TEST_F(UserVfsTest, BufferPath) {
  mjVFS vfs;
  mj_defaultVFS(&vfs);
    std::string buffer = "<mujoco/>";
    const void* ptr = static_cast<const void*>(buffer.c_str());
  mj_addBufferVFS(&vfs, "dir/model", ptr, buffer.size());
  EXPECT_TRUE(HasFile(&vfs, "files/../dir/model"));
  mj_deleteVFS(&vfs);
}

TEST_F(UserVfsTest, Timestamps) {
  static constexpr char cube[] = R"(
  v -0.500000 -0.500000  0.500000
  v  0.500000 -0.500000  0.500000
  v -0.500000  0.500000  0.500000
  v  0.500000  0.500000  0.500000
  v -0.500000  0.500000 -0.500000
  v  0.500000  0.500000 -0.500000
  v -0.500000 -0.500000 -0.500000
  v  0.500000 -0.500000 -0.500000)";

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "cube.obj", cube, sizeof(cube));

  mjResource* resource = mju_openResource("", "cube.obj", &vfs, nullptr, 0);

  // same timestamps
  EXPECT_EQ(mju_isModifiedResource(resource, resource->timestamp), 0);

  // different timestamps
  EXPECT_EQ(mju_isModifiedResource(resource, "QQ=="), 1);

  mju_closeResource(resource);
  mj_deleteVFS(&vfs);
}

}  // namespace
}  // namespace mujoco
