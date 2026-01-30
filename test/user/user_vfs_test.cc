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

using ::testing::IsNull;
using ::testing::NotNull;
using UserVfsTest = MujocoTest;

static bool HasFile(const mjVFS* vfs, const std::string& filename) {
  mjResource* resource = mju_openResource("", filename.c_str(), vfs, nullptr, 0);
  bool result = resource != nullptr;
  mju_closeResource(resource);
  return result;
}

struct TestProvider : public mjpResourceProvider {
  // The TestProvider will increment the value at `addr` by these amounts when
  // each callback is invoked. This can be used to verity that the correct
  // callbacks are being invoked when expected.
  enum CallbackValues {
    Mounted = 100,
    Unmounted = 200,
    Opened = 300,
    Read = 400,
    Closed = 500,
  };

  explicit TestProvider(int* addr) {
    mjp_defaultResourceProvider(this);
    prefix = "test";
    data = addr;

    mount = [](mjResource* res) {
      *(int*)res->provider->data += Mounted;
      return 1;
    };
    unmount = [](mjResource* res) {
      *(int*)res->provider->data += Unmounted;
      return 1;
    };
    open = [](mjResource* res) {
      *(int*)res->provider->data += Opened;
      return 1;
    };
    read = [](mjResource* res, const void** out) {
      *(int*)res->provider->data += Read;
      *out = res->provider->data;
      return (int)sizeof(int);
    };
    close = [](mjResource* res) {
      *(int*)res->provider->data += Closed;
    };
  }
};

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

TEST_F(UserVfsTest, NullDirectory) {
  mjVFS vfs;
  mj_defaultVFS(&vfs);

  const std::string file = "engine/testdata/actuation/activation.xml";
  const std::string path = GetTestDataFilePath(file);
  mj_addFileVFS(&vfs, nullptr, path.c_str());

  EXPECT_TRUE(HasFile(&vfs, "activation.xml"));
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

TEST_F(UserVfsTest, MountUnmount) {
  int test = 0;
  int expect = 0;
  TestProvider provider(&test);

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mjResource* res = mju_openResource("", "/some/path/foo", &vfs, nullptr, 0);
  EXPECT_THAT(res, IsNull());
  EXPECT_EQ(test, expect);

  mj_mountVFS(&vfs, "/some/path", &provider);
  expect += TestProvider::Mounted;
  EXPECT_EQ(test, expect);

  res = mju_openResource("", "/some/path/foo", &vfs, nullptr, 0);
  expect += TestProvider::Opened;
  EXPECT_THAT(res, NotNull());
  EXPECT_EQ(test, expect);

  const void* buffer = nullptr;
  const int size = mju_readResource(res, &buffer);
  expect += TestProvider::Read;
  EXPECT_THAT(buffer, NotNull());
  EXPECT_GT(size, 0);
  EXPECT_EQ(test, expect);

  mju_closeResource(res);
  expect += TestProvider::Closed;
  EXPECT_EQ(test, expect);

  mj_unmountVFS(&vfs, "/some/path");
  expect += TestProvider::Unmounted;
  EXPECT_EQ(test, expect);

  mj_deleteVFS(&vfs);
}

TEST_F(UserVfsTest, AutoMountProviders) {
  int test = 0;
  int expect = 0;

  TestProvider provider(&test);
  mjp_registerResourceProvider(&provider);

  mjVFS vfs;
  mj_defaultVFS(&vfs);

  mjResource* res = mju_openResource("", "test:foo", &vfs, nullptr, 0);
  expect += TestProvider::Mounted + TestProvider::Opened;
  EXPECT_THAT(res, NotNull());

  mju_closeResource(res);
  expect += TestProvider::Closed;
  EXPECT_EQ(test, expect);

  mj_deleteVFS(&vfs);
  expect += TestProvider::Unmounted;
  EXPECT_EQ(test, expect);
}

TEST_F(UserVfsTest, StackedMounts) {
  int test1 = 0;
  int test2 = 1000;
  int test3 = 1000000;
  int expect1 = test1;
  int expect2 = test2;
  int expect3 = test3;

  TestProvider provider1(&test1);
  TestProvider provider2(&test2);
  TestProvider provider3(&test3);

  mjVFS vfs;
  mj_defaultVFS(&vfs);

  mj_mountVFS(&vfs, "/some/path", &provider1);
  mj_mountVFS(&vfs, "/some/path/further/down/very/deep", &provider2);
  mj_mountVFS(&vfs, "/some/path/further/down", &provider3);
  expect1 += TestProvider::Mounted;
  expect2 += TestProvider::Mounted;
  expect3 += TestProvider::Mounted;
  EXPECT_EQ(test1, expect1);
  EXPECT_EQ(test2, expect2);
  EXPECT_EQ(test3, expect3);

  mjResource* res1 = mju_openResource("", "/some/path/foo", &vfs, nullptr, 0);
  expect1 += TestProvider::Opened;
  EXPECT_EQ(test1, expect1);
  EXPECT_EQ(test2, expect2);
  EXPECT_EQ(test3, expect3);

  mjResource* res2 =
      mju_openResource("", "/some/path/further/down/foo", &vfs, nullptr, 0);
  expect3 += TestProvider::Opened;
  EXPECT_EQ(test1, expect1);
  EXPECT_EQ(test2, expect2);
  EXPECT_EQ(test3, expect3);

  mjResource* res3 = mju_openResource(
      "", "/some/path/further/down/very/deep/foo", &vfs, nullptr, 0);
  expect2 += TestProvider::Opened;
  EXPECT_EQ(test1, expect1);
  EXPECT_EQ(test2, expect2);
  EXPECT_EQ(test3, expect3);

  mju_closeResource(res1);
  mju_closeResource(res2);
  mju_closeResource(res3);
  expect1 += TestProvider::Closed;
  expect2 += TestProvider::Closed;
  expect3 += TestProvider::Closed;

  mj_deleteVFS(&vfs);
  expect1 += TestProvider::Unmounted;
  expect2 += TestProvider::Unmounted;
  expect3 += TestProvider::Unmounted;
  EXPECT_EQ(test1, expect1);
  EXPECT_EQ(test2, expect2);
  EXPECT_EQ(test3, expect3);
}
}  // namespace
}  // namespace mujoco
