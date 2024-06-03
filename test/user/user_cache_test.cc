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

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include "test/fixture.h"
#include "src/user/user_cache.h"

namespace mujoco {

using ::testing::ElementsAreArray;
using ::testing::IsNull;
using ::testing::NotNull;
using ::testing::StrEq;

using CacheTest = MujocoTest;

namespace {

constexpr int kMaxSize = 100;  // in bytes

TEST(CacheTest, SizeTest) {
  mjCCache cache(kMaxSize);
  EXPECT_EQ(cache.Size(), 0);
}

TEST(CacheTest, HasAssetSuccessTest) {
  mjCCache cache(kMaxSize);

  mjCAsset asset("file.xml", "foo.obj", "now");
  cache.Insert(asset);

  EXPECT_THAT(*(cache.HasAsset("foo.obj")), StrEq("now"));
}

TEST(CacheTest, HasAssetFailureTest) {
  mjCCache cache(kMaxSize);

  mjCAsset asset("file.xml", "foo.obj", "now");
  cache.Insert(asset);

  EXPECT_THAT(cache.HasAsset("file2.xml"), nullptr);
}

TEST(CacheTest, AddSuccessTest) {
  mjCCache cache(kMaxSize);
  std::vector<int> v1 = {1, 2, 3};
  std::vector<double> v2 = {1.0, 2.0, 3.0};

  mjCAsset asset("file.xml", "foo.obj", "now");
  std::size_t nbytes1 = asset.AddVector("v1", v1);
  std::size_t nbytes2 = asset.AddVector("v2", v2);
  cache.Insert(asset);

  ASSERT_EQ(nbytes1, 12);
  ASSERT_EQ(nbytes2, 24);
  ASSERT_EQ(cache.Size(), 36);
}

TEST(CacheTest, AddFailureTest) {
  mjCCache cache(kMaxSize);
  std::vector<int> v1 = {1, 2, 3};
  std::vector<double> v2 = {1.0, 2.0, 3.0};

  mjCAsset asset("file.xml", "foo.obj", "now");
  asset.AddVector("v1", v1);
  std::size_t nbytes = asset.AddVector("v1", v2);
  cache.Insert(asset);

  ASSERT_EQ(nbytes, 0);
  ASSERT_EQ(cache.Size(), 12);
}

TEST(CacheTest, InsertReplaceTest) {
  mjCCache cache(kMaxSize);
  std::vector<int> v1 = {1, 2, 3};
  std::vector<double> v2 = {1.0, 2.0, 3.0};

  mjCAsset asset("file.xml", "foo.obj", "now");
  asset.AddVector("v", v1);
  cache.Insert(asset);

  mjCAsset asset2("file.xml", "foo.obj", "nower");
  asset2.AddVector("v", v2);
  bool inserted = cache.Insert(asset2);
  EXPECT_TRUE(inserted);

  mjCAsset asset3 = *(cache.Get("foo.obj"));
  std::vector<double> v3 = asset3.GetVector<double>("v").value();
  EXPECT_THAT(v3, ElementsAreArray(v2));

  ASSERT_EQ(cache.Size(), 24);
}

TEST(CacheTest, MoveInsertNewTest) {
  mjCCache cache(kMaxSize);
  std::vector<int> v1 = {1, 2, 3};
  std::vector<double> v2 = {1.0, 2.0, 3.0};

  mjCAsset asset("file.xml", "foo.obj", "now");
  std::size_t nbytes1 = asset.AddVector("v1", v1);
  std::size_t nbytes2 = asset.AddVector("v2", v2);
  cache.Insert(std::move(asset));

  ASSERT_EQ(nbytes1, 12);
  ASSERT_EQ(nbytes2, 24);
  ASSERT_EQ(cache.Size(), 36);
}

TEST(CacheTest, MoveInsertReplaceTest) {
  mjCCache cache(kMaxSize);
  std::vector<int> v1 = {1, 2, 3};
  std::vector<double> v2 = {1.0, 2.0, 3.0};

  mjCAsset asset("file.xml", "foo.obj", "now");
  asset.AddVector("v", v1);
  cache.Insert(std::move(asset));

  mjCAsset asset2("file.xml", "foo.obj", "nower");
  asset2.AddVector("v", v2);
  bool inserted = cache.Insert(std::move(asset2));
  EXPECT_TRUE(inserted);

  mjCAsset asset3 = *(cache.Get("foo.obj"));
  std::vector<double> v3 = asset3.GetVector<double>("v").value();
  EXPECT_THAT(v3, ElementsAreArray(v2));

  ASSERT_EQ(cache.Size(), 24);
}

TEST(CacheTest, GetSuccessTest) {
  mjCCache cache(kMaxSize);
  std::vector<int> v1 = {1, 2, 3};
  std::vector<double> v2 = {4.0, 5.0, 6.0};
  mjCAsset asset("file.xml", "foo.obj", "now");
  asset.AddVector("v1", v1);
  asset.AddVector("v2", v2);
  cache.Insert(asset);

  mjCAsset asset2 = *(cache.Get("foo.obj"));

  std::vector<int> v3 = asset2.GetVector<int>("v1").value();
  EXPECT_THAT(v3, ElementsAreArray(v1));

  std::vector<double> v4 = asset2.GetVector<double>("v2").value();
  EXPECT_THAT(v4, ElementsAreArray(v2));
}

TEST(CacheTest, GetFailueTest) {
  mjCCache cache(kMaxSize);
  std::vector<int> v = {1, 2, 3};
  mjCAsset asset("file.xml", "foo.obj", "now");
  asset.AddVector("v", v);
  cache.Insert(asset);

  mjCAsset asset2 = *(cache.Get("foo.obj"));
  EXPECT_EQ(cache.Get("bar.obj").has_value(), false);

  auto v2 = asset2.GetVector<int>("v2");
  EXPECT_EQ(v2.has_value(), false);
}

// Trim cache based off of access count
TEST(CacheTest, LimitTest1) {
  mjCCache cache(kMaxSize);
  EXPECT_THAT(cache.MaxSize(), kMaxSize);
  std::vector<int> v = {1, 2, 3};

  mjCAsset asset1 = mjCAsset("file.xml", "foo.obj", "now");
  mjCAsset asset2 = mjCAsset("file.xml", "bar.obj", "now");
  asset1.AddVector("foo.obj", v);
  asset2.AddVector("bar.obj", v);
  cache.Insert(asset1);
  cache.Insert(asset2);

  // access asset foo twice, bar one
  cache.Get("foo.obj");
  cache.Get("foo.obj");
  cache.Get("bar.obj");

  // make max size so cache can hold only one asset
  cache.SetMaxSize(12);

  // foo should still be in cache
  EXPECT_THAT(cache.HasAsset("foo.obj"), NotNull());

  // bar was accessed less, so is removed
  EXPECT_THAT(cache.HasAsset("bar.obj"), IsNull());
}

// Trim cache based off of insert order
TEST(CacheTest, LimitTest2) {
  mjCCache cache(kMaxSize);
  std::vector<int> v = {1, 2, 3};
  mjCAsset asset1("file.xml", "foo.obj", "now");
  mjCAsset asset2("file.xml", "bar.obj", "now");
  asset1.AddVector("v", v);
  asset2.AddVector("v", v);

  cache.Insert(asset1);
  cache.Insert(asset2);

  // get each asset once
  mjCAsset asset3 = *(cache.Get("foo.obj"));
  mjCAsset asset4 = *(cache.Get("bar.obj"));

  // make max size so cache can hold only one asset
  cache.SetMaxSize(12);

  // foo should be gone because it's older
  EXPECT_THAT(cache.HasAsset("foo.obj"), IsNull());

  // bar should still be in cache
  EXPECT_THAT(cache.HasAsset("bar.obj"), NotNull());
}

// stress test with large asset
TEST(CacheTest, LimitTest3) {
  mjCCache cache(12);
  std::vector<int> v1 = {1, 2, 3};
  std::vector<int> v2 = {1, 2, 3, 4, 5};
  mjCAsset asset1("file.xml", "foo.obj", "now");
  mjCAsset asset2("file.xml", "bar.obj", "now");
  asset1.AddVector("v", v1);
  asset2.AddVector("v", v2);

  cache.Insert(std::move(asset1));
  cache.Insert(std::move(asset2));

  // foo should still be in cache
  EXPECT_THAT(cache.HasAsset("foo.obj"), NotNull());

  // bar could not be inserted because it's too large
  EXPECT_THAT(cache.HasAsset("bar.obj"), IsNull());
}

TEST(CacheTest, LimitTest4) {
  mjCCache cache(12);
  std::vector<int> v = {1, 2, 3};
  mjCAsset asset1("file.xml", "foo.obj", "now");
  mjCAsset asset2("file.xml", "bar.obj", "now");
  asset1.AddVector("v", v);
  asset2.AddVector("v", v);

  cache.Insert(std::move(asset1));
  cache.Insert(std::move(asset2));

  EXPECT_THAT(cache.HasAsset("foo.obj"), NotNull());

  // cache is full, so bar can't be inserted
  EXPECT_THAT(cache.HasAsset("bar.obj"), IsNull());
}

TEST(CacheTest, ResetAllTest) {
  mjCCache cache(kMaxSize);
  mjCAsset asset1("file1.xml", "foo.obj", "now");
  mjCAsset asset2("file2.xml", "bar.obj", "now");
  cache.Insert(asset1);
  cache.Insert(asset2);

  EXPECT_THAT(cache.HasAsset("foo.obj"), NotNull());
  EXPECT_THAT(cache.HasAsset("bar.obj"), NotNull());

  cache.Reset();

  EXPECT_THAT(cache.HasAsset("foo.obj"), IsNull());
  EXPECT_THAT(cache.HasAsset("bar.obj"), IsNull());
}

TEST(CacheTest, ResetModelTest1) {
  mjCCache cache(kMaxSize);
  mjCAsset asset("file1.xml", "foo.obj", "now");
  mjCAsset asset2("file1.xml", "bar.obj", "now");
  mjCAsset asset3("file2.xml", "bar.obj", "now");
  cache.Insert(asset);
  cache.Insert(asset2);
  cache.Insert(asset3);

  cache.Reset("file2.xml");

  EXPECT_THAT(cache.HasAsset("foo.obj"), NotNull());
  EXPECT_THAT(cache.HasAsset("bar.obj"), IsNull());
}

TEST(CacheTest, ResetModelTest2) {
  mjCCache cache(kMaxSize);
  mjCAsset asset("file1.xml", "foo.obj", "now");
  mjCAsset asset2("file2.xml", "foo.obj", "now");
  mjCAsset asset3("file2.xml", "bar.obj", "now");
  cache.Insert(asset);
  cache.Insert(asset2);
  cache.Insert(asset3);

  cache.Reset("file2.xml");

  EXPECT_THAT(cache.HasAsset("foo.obj"), IsNull());
  EXPECT_THAT(cache.HasAsset("bar.obj"), IsNull());
}

TEST(CacheTest, RemoveModelTest1) {
  mjCCache cache(kMaxSize);
  mjCAsset asset("file1.xml", "foo.obj", "now");
  mjCAsset asset2("file1.xml", "bar.obj", "now");
  mjCAsset asset3("file2.xml", "bar.obj", "now");
  cache.Insert(asset);
  cache.Insert(asset2);
  cache.Insert(asset3);

  cache.RemoveModel("file2.xml");

  EXPECT_THAT(cache.HasAsset("foo.obj"), NotNull());
  EXPECT_THAT(cache.HasAsset("bar.obj"), NotNull());
}

TEST(CacheTest, RemoveModelTest2) {
  mjCCache cache(kMaxSize);
  mjCAsset asset("file1.xml", "foo.obj", "now");
  mjCAsset asset2("file2.xml", "bar.obj", "now");
  cache.Insert(asset);
  cache.Insert(asset2);

  cache.Reset("file2.xml");

  EXPECT_THAT(cache.HasAsset("foo.obj"), NotNull());
  EXPECT_THAT(cache.HasAsset("bar.obj"), IsNull());
}

TEST(CacheTest, DeleteAssetSuccessTest) {
  mjCCache cache(kMaxSize);
  mjCAsset asset("file1.xml", "foo.obj", "now");
  cache.Insert(asset);

  cache.DeleteAsset("foo.obj");

  EXPECT_THAT(cache.HasAsset("foo.obj"), IsNull());
}

TEST(CacheTest, DeleteAssetFailureTest) {
  mjCCache cache(kMaxSize);
  mjCAsset asset("file.xml", "foo.obj", "now");
  cache.Insert(asset);

  cache.DeleteAsset("bar.obj");

  EXPECT_THAT(cache.HasAsset("foo.obj"), NotNull());
}

}  // namespace
}  // namespace mujoco
