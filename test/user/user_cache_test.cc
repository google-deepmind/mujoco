// Copyright 2024 DeepMind Technologies Limited
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

// Tests for user/user_cache.cc

#include <cstring>
#include <memory>
#include <string>
#include <optional>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"
#include "src/user/user_cache.h"
#include "src/user/user_vfs.h"
#include "src/user/user_resource.h"

namespace mujoco {

using ::testing::IsNull;
using ::testing::NotNull;
using ::testing::StrEq;

using CacheTest = MujocoTest;

namespace {

constexpr int kMaxSize = 100;  // in bytes
constexpr std::string kText = "Hello World";
constexpr std::string kTestId = "Test";
constexpr std::string kModel = "myModel";
constexpr std::string kFile = "hello.txt";

void CacheText(mjCCache& cache, const std::string& id, const std::string& model,
              const std::string& name, const std::string& text) {
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, name.c_str(), text.data(), text.size());
  mjResource* resource = mju_openResource("", name.c_str(), &vfs, nullptr, 0);
  std::shared_ptr<const void> data(&text, +[](const void* data) {});
  cache.Insert(model, id, resource, data, text.size());
  mju_closeResource(resource);
  mj_deleteVFS(&vfs);
}

std::optional<std::string>
GetCachedText(mjCCache& cache, const std::string& id, const std::string& model,
              const std::string& name, const std::string& text) {
  std::string cached_text;
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, name.c_str(), text.data(), std::strlen(text.c_str()));
  mjResource* resource = mju_openResource("", name.c_str(), &vfs, nullptr, 0);
  bool inserted = cache.PopulateData(id, resource,
                                     [&cached_text](const void* data) {
    cached_text = *(static_cast<const std::string*>(data));
    return true;
  });
  mju_closeResource(resource);
  mj_deleteVFS(&vfs);
  return inserted ? std::optional<std::string>(cached_text) : std::nullopt;
}


TEST(CacheTest, SizeTest) {
  mjCCache cache(kMaxSize);
  EXPECT_EQ(cache.Size(), 0);
}

TEST(CacheTest, InsertSuccess) {
  mjCCache cache(kMaxSize);
  CacheText(cache, kTestId, kModel, kFile, kText);
  auto cached_text = GetCachedText(cache, kTestId, kModel, kFile, kText);

  EXPECT_THAT(cached_text.value(), StrEq(kText));
}

TEST(CacheTest, InsertFailure) {
  mjCCache cache(kMaxSize);
  CacheText(cache, kTestId, kModel, kFile, kText);
  auto cached_text = GetCachedText(
      cache, "WrongId", kModel, "hello2.txt", kText);
  EXPECT_EQ(cached_text, std::nullopt);
}

TEST(CacheTest, InsertReplace) {
  mjCCache cache(kMaxSize);
  const std::string kUpdatedText = "Goodbye World";
  CacheText(cache, kTestId, kModel, kFile, kText);
  CacheText(cache, kTestId, kModel, kFile, kUpdatedText);
  auto cached_text = GetCachedText(cache, kTestId, kModel, kFile, kUpdatedText);

  EXPECT_THAT(cached_text.value(), StrEq(kUpdatedText));
}

// Trim cache based off of access count
TEST(CacheTest, Limit1) {
  mjCCache cache(kMaxSize);
  EXPECT_THAT(cache.Capacity(), kMaxSize);

  CacheText(cache, "foo", "fil.xml", "foo.obj", kText);
  CacheText(cache, "bar", "fil.xml", "bar.obj", kText);

  // access asset foo twice, bar one
  GetCachedText(cache, "foo", "file.xml", "foo.obj", kText);
  GetCachedText(cache, "foo", "file.xml", "foo.obj", kText);
  GetCachedText(cache, "bar", "file.xml", "bar.obj", kText);

  // make max size so cache can hold only one asset
  cache.SetCapacity(12);

  // foo should still be in cache
  EXPECT_THAT(cache.HasAsset("foo"), NotNull());

  // bar was accessed less, so is removed
  EXPECT_THAT(cache.HasAsset("bar"), IsNull());
}


// Trim cache based off of insert order
TEST(CacheTest, Limit2) {
  mjCCache cache(kMaxSize);
  CacheText(cache, "foo", "file.xml", "foo.obj", kText);
  CacheText(cache, "bar", "file.xml", "bar.obj", kText);

  // get each asset once
  GetCachedText(cache, "foo", "file.xml", "foo.obj", kText);
  GetCachedText(cache, "bar", "file.xml", "bar.obj", kText);

  // make max size so cache can hold only one asset
  cache.SetCapacity(12);

  // foo should be gone because it's older
  EXPECT_THAT(cache.HasAsset("foo"), IsNull());

  // bar should still be in cache
  EXPECT_THAT(cache.HasAsset("bar"), NotNull());
}

// stress test with large asset
TEST(CacheTest, Limit3) {
  mjCCache cache(12);
  CacheText(cache, "foo", "file.xml", "foo.obj", kText);
  CacheText(cache, "bar", "file.xml", "bar.obj", kText);

  // foo should still be in cache
  EXPECT_THAT(cache.HasAsset("foo"), NotNull());

  // bar could not be inserted because it's too large
  EXPECT_THAT(cache.HasAsset("bar"), IsNull());
}

TEST(CacheTest, Limit4) {
  mjCCache cache(12);
  CacheText(cache, "foo", "file.xml", "foo.obj", kText);
  CacheText(cache, "bar", "file.xml", "bar.obj", kText);

  EXPECT_THAT(cache.HasAsset("foo"), NotNull());

  // cache is full, so bar can't be inserted
  EXPECT_THAT(cache.HasAsset("bar"), IsNull());
}

TEST(CacheTest, ResetAll) {
  mjCCache cache(kMaxSize);
  CacheText(cache, "foo", "file1.xml", "foo.obj", kText);
  CacheText(cache, "bar", "file2.xml", "bar.obj", kText);

  EXPECT_THAT(cache.HasAsset("foo"), NotNull());
  EXPECT_THAT(cache.HasAsset("bar"), NotNull());

  cache.Reset();

  EXPECT_THAT(cache.HasAsset("foo"), IsNull());
  EXPECT_THAT(cache.HasAsset("bar"), IsNull());
}

TEST(CacheTest, ResetModel1) {
  mjCCache cache(kMaxSize);
  CacheText(cache, "foo", "file1.xml", "foo.obj", kText);
  CacheText(cache, "bar", "file1.xml", "bar.obj", kText);
  CacheText(cache, "bar", "file2.xml", "bar.obj", kText);

  cache.Reset("file2.xml");

  EXPECT_THAT(cache.HasAsset("foo"), NotNull());
  EXPECT_THAT(cache.HasAsset("bar"), IsNull());
}

TEST(CacheTest, ResetModel2) {
  mjCCache cache(kMaxSize);
  CacheText(cache, "foo", "file1.xml", "foo.obj", kText);
  CacheText(cache, "foo", "file2.xml", "foo.obj", kText);
  CacheText(cache, "bar", "file2.xml", "bar.obj", kText);

  cache.Reset("file2.xml");

  EXPECT_THAT(cache.HasAsset("foo"), IsNull());
  EXPECT_THAT(cache.HasAsset("bar"), IsNull());
}

TEST(CacheTest, RemoveModel1) {
  mjCCache cache(kMaxSize);
  CacheText(cache, "foo", "file1.xml", "foo.obj", kText);
  CacheText(cache, "bar", "file1.xml", "bar.obj", kText);
  CacheText(cache, "bar", "file2.xml", "bar.obj", kText);

  cache.RemoveModel("file2.xml");

  EXPECT_THAT(cache.HasAsset("foo"), NotNull());
  EXPECT_THAT(cache.HasAsset("bar"), NotNull());
}

TEST(CacheTest, RemoveModel2) {
  mjCCache cache(kMaxSize);
  CacheText(cache, "foo", "file1.xml", "foo.obj", kText);
  CacheText(cache, "bar", "file2.xml", "bar.obj", kText);

  cache.Reset("file2.xml");

  EXPECT_THAT(cache.HasAsset("foo"), NotNull());
  EXPECT_THAT(cache.HasAsset("bar"), IsNull());
}

TEST(CacheTest, DeleteAssetSuccess) {
  mjCCache cache(kMaxSize);
  CacheText(cache, "foo", "file.xml", "foo.obj", kText);

  cache.DeleteAsset("foo");

  EXPECT_THAT(cache.HasAsset("foo"), IsNull());
}

TEST(CacheTest, DeleteAssetFailure) {
  mjCCache cache(kMaxSize);
  CacheText(cache, "foo", "file.xml", "foo.obj", kText);

  cache.DeleteAsset("bar");

  EXPECT_THAT(cache.HasAsset("foo"), NotNull());
}

}  // namespace
}  // namespace mujoco
