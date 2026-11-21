// Copyright 2026 DeepMind Technologies Limited
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

#include "test/compare_model.h"

#include <string>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using CompareModelTest = MujocoTest;

constexpr char kModel[] = R"(
<mujoco>
  <worldbody>
    <body>
      <joint/>
      <geom type="sphere" size=".1" rgba="0 0 0 1"/>
    </body>
  </worldbody>
</mujoco>
)";

TEST_F(CompareModelTest, EqualModelsPreserveField) {
  MjModelPtr model = LoadModelFromString(kModel);
  ASSERT_NE(model, nullptr);
  MjModelPtr copy(mj_copyModel(nullptr, model.get()));
  ASSERT_NE(copy, nullptr);
  std::string field = "unchanged";
  EXPECT_EQ(CompareModel(model.get(), copy.get(), field), 0);
  EXPECT_EQ(field, "unchanged");
}

TEST_F(CompareModelTest, ArrayTypesAndCoordinates) {
  MjModelPtr model = LoadModelFromString(kModel);
  ASSERT_NE(model, nullptr);
  struct Case {
    void (*edit)(mjModel*);
    mjtNum difference;
    const char* field;
  };
  const Case cases[] = {
      {[](mjModel* m) { m->body_pos[5] = .25; }, .25, "body_pos row: 1 col: 2"},
      {[](mjModel* m) { m->geom_group[0] = 2; }, 2, "geom_group row: 0 col: 0"},
      {[](mjModel* m) { m->body_simple[1] = !m->body_simple[1]; }, 1,
       "body_simple row: 1 col: 0"},
      {[](mjModel* m) { m->names[0] = m->names[0] == 'x' ? 'y' : 'x'; }, 1,
       "names row: 0 col: 0"},
      {[](mjModel* m) { m->geom_rgba[0] = .25; }, .25,
       "geom_rgba row: 0 col: 0"},
  };
  for (const Case& test : cases) {
    SCOPED_TRACE(test.field);
    MjModelPtr copy(mj_copyModel(nullptr, model.get()));
    ASSERT_NE(copy, nullptr);
    test.edit(copy.get());
    std::string field;
    EXPECT_EQ(CompareModel(model.get(), copy.get(), field), test.difference);
    EXPECT_EQ(field, test.field);
  }
}

TEST_F(CompareModelTest, LargestDifferenceAndFirstTie) {
  MjModelPtr model = LoadModelFromString(kModel);
  ASSERT_NE(model, nullptr);
  MjModelPtr copy(mj_copyModel(nullptr, model.get()));
  ASSERT_NE(copy, nullptr);
  copy->body_pos[4] = .25;
  copy->body_pos[5] = .25;
  copy->geom_pos[0] = .25;
  std::string field;
  EXPECT_EQ(CompareModel(model.get(), copy.get(), field), .25);
  EXPECT_EQ(field, "body_pos row: 1 col: 1");
  copy->geom_pos[0] = .5;
  EXPECT_EQ(CompareModel(model.get(), copy.get(), field), .5);
  EXPECT_EQ(field, "geom_pos row: 0 col: 0");
}

}  // namespace
}  // namespace mujoco
