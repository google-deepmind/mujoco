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

#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "src/xml/xml_util.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using XMLUtilTest = MujocoTest;

using ::testing::ElementsAre;

TEST_F(XMLUtilTest, String2VectorFloat) {
  std::vector<float> v = mjXUtil::String2Vector<float>(" 1.2 3.2     5.3 6 ");
  EXPECT_THAT(v, ElementsAre(1.2, 3.2, 5.3, 6));
}

TEST_F(XMLUtilTest, String2VectorEmpty) {
  std::vector<float> v = mjXUtil::String2Vector<float>("");
  EXPECT_THAT(v, ElementsAre());
}

TEST_F(XMLUtilTest, String2VectorError) {
  std::vector<float> v = mjXUtil::String2Vector<float>("ABCD. /123/122/113");
  EXPECT_THAT(v, ElementsAre());
}


TEST_F(XMLUtilTest, String2VectorInt) {
  std::vector<int> v = mjXUtil::String2Vector<int>(" -1 3 5 6");
  EXPECT_THAT(v, ElementsAre(-1, 3, 5, 6));
}

TEST_F(XMLUtilTest, String2VectorString) {
  auto v = mjXUtil::String2Vector<std::string>(" abc  def ");
  EXPECT_THAT(v, ElementsAre("abc", "def"));
}

}  // namespace
}  // namespace mujoco
