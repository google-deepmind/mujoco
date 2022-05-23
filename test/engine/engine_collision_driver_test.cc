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

// Tests for engine/engine_collision_driver.c.

#include <cstddef>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using MjCollisionTest = MujocoTest;
using GeomPair = std::pair<std::string, std::string>;
using ::testing::IsEmpty;
using ::testing::ElementsAre;

// Returns a sorted list of pairs of colliding geom names, where each pair of
// geom names is sorted.
static std::vector<GeomPair> colliding_pairs(
    const mjModel* model, const mjData* data) {
  std::vector<GeomPair> result;
  for (int i = 0; i < data->ncon; i++) {
    std::string geom1 = mj_id2name(model, mjOBJ_GEOM, data->contact[i].geom1);
    std::string geom2 = mj_id2name(model, mjOBJ_GEOM, data->contact[i].geom2);
    result.push_back(GeomPair(std::min(geom1, geom2), std::max(geom1, geom2)));
  }
  std::sort(result.begin(), result.end());
  return result;
}

TEST_F(MjCollisionTest, PredefinedPairsOnly) {
  static const char* const kModelFilePath =
      "engine/testdata/collisions.xml";
  const std::string xml_path = GetTestDataFilePath(kModelFilePath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);

  model->opt.collision = mjCOL_PAIR;
  mjData* data = mj_makeData(model);
  mj_fwdPosition(model, data);
  EXPECT_THAT(colliding_pairs(model, data), ElementsAre(
      GeomPair("box", "sphere_predefined")));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjCollisionTest, AllCollisions) {
  static const char* const kModelFilePath =
      "engine/testdata/collisions.xml";
  const std::string xml_path = GetTestDataFilePath(kModelFilePath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  mjData* data = mj_makeData(model);

  // mjCOL_ALL is the default
  mj_fwdPosition(model, data);
  EXPECT_THAT(colliding_pairs(model, data), ElementsAre(
      GeomPair("box", "sphere_collides"),
      GeomPair("box", "sphere_predefined")
  ));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjCollisionTest, EmptyModel) {
  mjModel* model = LoadModelFromString("<mujoco/>");
  mjData* data = mj_makeData(model);

  mj_fwdPosition(model, data);
  EXPECT_THAT(colliding_pairs(model, data), IsEmpty());

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjCollisionTest, ZeroedHessian) {
  static const char* const kModelFilePath =
      "engine/testdata/collisions.xml";
  const std::string xml_path = GetTestDataFilePath(kModelFilePath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  mjData* data = mj_makeData(model);

  mj_fwdPosition(model, data);
  for (int i = 0; i < data->ncon; i++) {
    for (int j = 0; j < 36; j++) {
      EXPECT_FALSE(isnan(data->contact[i].H[j]))
          << "NaN in contact[" << i << "].H[" << j << "]";
    }
  }
  mj_deleteData(data);
  mj_deleteModel(model);
}
}  // namespace
}  // namespace mujoco
