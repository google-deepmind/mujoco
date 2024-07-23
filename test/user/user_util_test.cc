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

#include <cerrno>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using user::FilePath;
using user::StringToVector;
using user::VectorToString;
using ::testing::ElementsAre;
using ::testing::IsNan;

using UserUtilTest = MujocoTest;

TEST_F(UserUtilTest, PathReduce) {
  FilePath path = FilePath("/hello/.././world/");
  EXPECT_EQ(path.Str(), "/world/");
}

TEST_F(UserUtilTest, PathReduce2) {
  FilePath path = FilePath("../hello/./world/");
  EXPECT_EQ(path.Str(), "../hello/world/");
}

TEST_F(UserUtilTest, PathReduce3) {
  FilePath path = FilePath("../../hello/world.txt");
  EXPECT_EQ(path.Str(), "../../hello/world.txt");
}

TEST_F(UserUtilTest, PathReduceWin) {
  FilePath path = FilePath("C:\\hello\\..\\world");
  EXPECT_EQ(path.Str(), "C:\\world");
}

TEST_F(UserUtilTest, IsAbs) {
  EXPECT_TRUE(FilePath("/hello").IsAbs());
  EXPECT_TRUE(FilePath("C:\\hello").IsAbs());
  EXPECT_FALSE(FilePath("hello").IsAbs());
}

TEST_F(UserUtilTest, Combine) {
  FilePath path1 = FilePath("/hello");
  FilePath path2 = FilePath("world");
  EXPECT_EQ((path1 + path2).Str(), "/hello/world");
}

TEST_F(UserUtilTest, Combine2) {
  FilePath path1 = FilePath("hello/");
  FilePath path2 = FilePath("world");
  EXPECT_EQ((path1 + path2).Str(), "hello/world");
}

TEST_F(UserUtilTest, Combine3) {
  FilePath path1 = FilePath("/hello");
  FilePath path2 = FilePath("../world");
  EXPECT_EQ((path1 + path2).Str(), "/world");
}

TEST_F(UserUtilTest, CombineAbs) {
  FilePath path1 = FilePath("/hello");
  FilePath path2 = FilePath("/world");
  EXPECT_EQ((path1 + path2).Str(), "/world");
}

TEST_F(UserUtilTest, Ext) {
  FilePath path = FilePath("/hello/world.txt");
  EXPECT_EQ(path.Ext(), ".txt");
}

TEST_F(UserUtilTest, ExtEmpty) {
  FilePath path = FilePath("/hello/world");
  EXPECT_EQ(path.Ext(), "");
}

TEST_F(UserUtilTest, StripExt) {
  FilePath path = FilePath("/hello/world.txt");
  EXPECT_EQ(path.StripExt().Str(), "/hello/world");
}

TEST_F(UserUtilTest, StripPath) {
  FilePath path = FilePath("/hello/world.txt");
  EXPECT_EQ(path.StripPath().Str(), "world.txt");
}

TEST_F(UserUtilTest, StripPathEmpty) {
  FilePath path = FilePath("world.txt");
  EXPECT_EQ(path.StripPath().Str(), "world.txt");
}

TEST_F(UserUtilTest, StripPathWin) {
  FilePath path = FilePath("\\world.txt");
  EXPECT_EQ(path.StripPath().Str(), "world.txt");
}


TEST_F(UserUtilTest, StrLower) {
  FilePath path = FilePath("/HELLO/worlD.txt");
  EXPECT_EQ(path.StrLower(), "/hello/world.txt");
}

TEST_F(UserUtilTest, StringToVectorFloat) {
  std::vector<float> v = StringToVector<float>(" 1.2 3.2     5.3 6 ");
  EXPECT_THAT(v, ElementsAre(1.2, 3.2, 5.3, 6));
  EXPECT_EQ(errno, 0);
}

TEST_F(UserUtilTest, StringToVectorEmpty) {
  std::vector<float> v = StringToVector<float>("   ");
  EXPECT_THAT(v, ElementsAre());
  EXPECT_EQ(errno, 0);
}

TEST_F(UserUtilTest, StringToVectorError) {
  std::vector<float> v = StringToVector<float>("2.1 3ABCD. /123/122/113");
  EXPECT_THAT(v, ElementsAre(2.1));
  EXPECT_EQ(errno, EINVAL);
}

TEST_F(UserUtilTest, StringToVectorInt) {
  std::vector<int> v = StringToVector<int>("  -1 3  5 6  ");
  EXPECT_THAT(v, ElementsAre(-1, 3, 5, 6));
  EXPECT_EQ(errno, 0);
}

TEST_F(UserUtilTest, StringToVectorString) {
  auto v = StringToVector<std::string>(" abc  def ");
  EXPECT_THAT(v, ElementsAre("abc", "def"));
}

TEST_F(UserUtilTest, StringToVectorInvalidNumber) {
  auto v = StringToVector<double>("1 0.1.2.3");
  EXPECT_THAT(v, ElementsAre(1));
  EXPECT_EQ(errno, EINVAL);
}

TEST_F(UserUtilTest, StringToVectorNan) {
  mju_user_warning = nullptr;
  auto v = StringToVector<double>("1 2 nan 3.21");
  EXPECT_THAT(v[2], IsNan());
  EXPECT_EQ(v[3], 3.21);
  EXPECT_EQ(errno, EDOM);
}

TEST_F(UserUtilTest, StringToVectorRange) {
  auto v = StringToVector<unsigned char>("-10");
  EXPECT_EQ(errno, ERANGE);
}

TEST_F(UserUtilTest, VectorToString) {
  std::vector<double> v = {1.2, 3.2, 5.3, 6};
  EXPECT_EQ(VectorToString(v), "1.2 3.2 5.3 6");
}

TEST_F(UserUtilTest, VectorToStringEmpty) {
  std::vector<double> v;
  EXPECT_EQ(VectorToString(v), "");
}

}  // namespace
}  // namespace mujoco
