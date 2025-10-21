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

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::NotNull;

static const char* const kModelPath = "testdata/model.xml";

class MjvSceneTest : public MujocoTest {
 protected:
  static constexpr int kMaxGeom = 10000;

  void InitSceneObjects(mjModel* model, int maxgeom = kMaxGeom) {
    mjv_defaultScene(&scn_);
    mjv_makeScene(model, &scn_, maxgeom);
    mjv_defaultOption(&opt_);
    mjv_defaultPerturb(&pert_);
    mjv_defaultFreeCamera(model, &cam_);

    // enable flags to exercise additional code paths
    for (int i = 0; i < mjNVISFLAG; ++i) {
      opt_.flags[i] = 1;
    }
  }

  void FreeSceneObjects() {
    mjv_freeScene(&scn_);
  }

  mjvScene scn_;
  mjvOption opt_;
  mjvPerturb pert_;
  mjvCamera cam_;
};

TEST_F(MjvSceneTest, UpdateScene) {
  const std::string xml_path = GetTestDataFilePath(kModelPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  ASSERT_THAT(model, NotNull()) << "Failed to load model from " << kModelPath;

  InitSceneObjects(model);

  mjData* data = mj_makeData(model);
  while (data->time < .2) {
    mj_step(model, data);
  }

  mjv_updateScene(model, data, &opt_, &pert_, &cam_, mjCAT_ALL, &scn_);
  EXPECT_EQ(scn_.status, 0);
  EXPECT_GT(scn_.ngeom, 0);
  EXPECT_GT(scn_.nlight, 0);
  if (model->nskin) EXPECT_GT(scn_.nskin, 0);
  if (model->nflex) EXPECT_GT(scn_.nflex, 0);

  mjv_updateScene(model, data, &opt_, &pert_, &cam_, mjCAT_ALL, &scn_);

  // call mj_copyData to expose any memory leaks in mjv_updateScene
  mjData* data_copy = mj_copyData(nullptr, model, data);

  mj_deleteData(data_copy);
  mj_deleteData(data);

  FreeSceneObjects();
  mj_deleteModel(model);
}

TEST_F(MjvSceneTest, UpdateSceneGeomsExhausted) {
  const std::string xml_path = GetTestDataFilePath(kModelPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  ASSERT_THAT(model, NotNull()) << "Failed to load model from " << kModelPath;

  const int maxgeoms = 1;
  InitSceneObjects(model, maxgeoms);

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  // clear handlers to avoid test failure; we are explicitly expecting a warning
  mju_clearHandlers();
  mjv_updateScene(model, data, &opt_, &pert_, &cam_, mjCAT_ALL, &scn_);
  EXPECT_EQ(scn_.status, 1);
  EXPECT_EQ(scn_.ngeom, maxgeoms);
  EXPECT_EQ(data->warning[mjWARN_VGEOMFULL].number, 1);

  mj_deleteData(data);
  FreeSceneObjects();
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
