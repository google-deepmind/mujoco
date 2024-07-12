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

// Tests for user/user_util.cc

#include "src/user/user_util.h"

#include <gtest/gtest.h>

namespace mujoco {
namespace {

using user::FilePath;

TEST(UserUtilTest, PathReduce) {
  FilePath path = FilePath("/hello/.././world/");
  EXPECT_EQ(path.Str(), "/world/");
}

TEST(UserUtilTest, PathReduce2) {
  FilePath path = FilePath("../hello/./world/");
  EXPECT_EQ(path.Str(), "../hello/world/");
}

TEST(UserUtilTest, PathReduceWin) {
  FilePath path = FilePath("C:\\hello\\..\\world");
  EXPECT_EQ(path.Str(), "C:\\world");
}

TEST(UserUtilTest, IsAbs) {
  EXPECT_TRUE(FilePath("/hello").IsAbs());
  EXPECT_TRUE(FilePath("C:\\hello").IsAbs());
  EXPECT_FALSE(FilePath("hello").IsAbs());
}

TEST(UserUtilTest, Combine) {
  FilePath path1 = FilePath("/hello");
  FilePath path2 = FilePath("world");
  EXPECT_EQ((path1 + path2).Str(), "/hello/world");
}

TEST(UserUtilTest, Combine2) {
  FilePath path1 = FilePath("hello/");
  FilePath path2 = FilePath("world");
  EXPECT_EQ((path1 + path2).Str(), "hello/world");
}

TEST(UserUtilTest, Combine3) {
  FilePath path1 = FilePath("/hello");
  FilePath path2 = FilePath("../world");
  EXPECT_EQ((path1 + path2).Str(), "/world");
}

TEST(UserUtilTest, CombineAbs) {
  FilePath path1 = FilePath("/hello");
  FilePath path2 = FilePath("/world");
  EXPECT_EQ((path1 + path2).Str(), "/world");
}

TEST(UserUtilTest, Ext) {
  FilePath path = FilePath("/hello/world.txt");
  EXPECT_EQ(path.Ext(), ".txt");
}

TEST(UserUtilTest, ExtEmpty) {
  FilePath path = FilePath("/hello/world");
  EXPECT_EQ(path.Ext(), "");
}

TEST(UserUtilTest, StripExt) {
  FilePath path = FilePath("/hello/world.txt");
  EXPECT_EQ(path.StripExt().Str(), "/hello/world");
}

TEST(UserUtilTest, StripPath) {
  FilePath path = FilePath("/hello/world.txt");
  EXPECT_EQ(path.StripPath().Str(), "world.txt");
}

TEST(UserUtilTest, StripPathEmpty) {
  FilePath path = FilePath("world.txt");
  EXPECT_EQ(path.StripPath().Str(), "world.txt");
}

TEST(UserUtilTest, StripPathWin) {
  FilePath path = FilePath("\\world.txt");
  EXPECT_EQ(path.StripPath().Str(), "world.txt");
}


TEST(UserUtilTest, StrLower) {
  FilePath path = FilePath("/HELLO/worlD.txt");
  EXPECT_EQ(path.StrLower(), "/hello/world.txt");
}

}  // namespace
}  // namespace mujoco
